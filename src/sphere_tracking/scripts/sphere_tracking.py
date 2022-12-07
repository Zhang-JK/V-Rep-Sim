import rospy
from std_msgs.msg import Bool
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

track = False
debug = False
speedPub = None

rp = 0.03
ri = 0.0001
rd = 0.001

tp = 0.02
ti = 0.0000
td = 0.0002

max_acce = 0.045
sphere_speed = 0.2

rotationIntegral = 0
transitionIntegral = 0
lastRotationError = 0
lastTransitionError = 0
lastTransitionSpeed = 0

def limitAcce(tranSpeed, maxAcce):
    global lastTransitionSpeed
    if tranSpeed > lastTransitionSpeed + maxAcce*7:
        lastTransitionSpeed = lastTransitionSpeed + maxAcce*7
    elif tranSpeed < lastTransitionSpeed - maxAcce:
        lastTransitionSpeed = lastTransitionSpeed - maxAcce
    else:
        lastTransitionSpeed = tranSpeed
    return lastTransitionSpeed


def pidPub(rotationError, transitionError):
    global rp, ri, rd, tp, ti, td
    global rotationIntegral, transitionIntegral
    global lastRotationError, lastTransitionError
    global speedPub

    rotationIntegral = rotationIntegral + rotationError
    transitionIntegral = transitionIntegral + transitionError

    rotationDerivative = rotationError - lastRotationError
    transitionDerivative = transitionError - lastTransitionError

    lastRotationError = rotationError
    lastTransitionError = transitionError

    rotationSpeed = rp * rotationError + ri * rotationIntegral + rd * rotationDerivative
    transitionSpeed = tp * transitionError + ti * transitionIntegral + td * transitionDerivative

    return rotationSpeed, transitionSpeed+sphere_speed
    

def triggerCallback(trigger):
    global track
    global rotationIntegral, transitionIntegral, lastRotationError, lastTransitionError
    track = trigger.data
    rotationIntegral = 0
    transitionIntegral = 0
    lastRotationError = 0
    lastTransitionError = 0
    rospy.loginfo(f'Auto tracking: {track}')


def imageCallback(image):
    if not track:
        cv2.destroyAllWindows()
        return
    raw = bridge.imgmsg_to_cv2(image, "bgr8")
    display = raw.copy()
    hsv_raw = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)
    lower = np.array([22, 93, 0], dtype="uint8")
    upper = np.array([45, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv_raw, lower, upper)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    maxArea = 900
    selected = None
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(display, (x, y), (x + w, y + h), (0, 0, 0), 3)
        if (w*h > maxArea):
            maxArea = w*h
            selected = c

    if selected is None:
        speed = Twist()
        speedPub.publish(speed) 
        cv2.waitKey(1)
        return

    x,y,w,h = cv2.boundingRect(selected)
    rotateError = x + w/2 - 256
    transitionError = w - 270
    rotationSpeed, transitionSpeed = pidPub(rotateError, transitionError)
    transitionSpeed = limitAcce(transitionSpeed, max_acce)
    cv2.putText(display, 'Error:', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
    cv2.putText(display, f'R: {rotateError}, T:{transitionError}', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 4)
    cv2.putText(display, 'Speed:', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
    cv2.putText(display, f'R: {round(rotationSpeed, 3)}, T:{round(transitionSpeed, 3)}', (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 4)
    display = cv2.resize(display, (256, 256), interpolation=cv2.INTER_CUBIC)
    cv2.imshow("Sphere Tracking" , display)
    speed = Twist()
    speed.linear.x = -transitionSpeed
    speed.angular.z = rotationSpeed
    speedPub.publish(speed)  
    
    if (debug):
        rospy.loginfo(f'rotateError: {rotateError}, transitionError: {transitionError}')
        rospy.loginfo(f'rotationSpeed: {rotationSpeed}, transitionSpeed: {transitionSpeed}')
        cv2.imshow("raw" , raw)
        cv2.imshow("mask" , mask)


    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('sphere_tracking')

    rp = rospy.get_param('~rota_p', rp)
    ri = rospy.get_param('~rota_i', ri)
    rd = rospy.get_param('~rota_d', rd)
    tp = rospy.get_param('~tran_p', tp)
    ti = rospy.get_param('~tran_i', ti)
    td = rospy.get_param('~tran_d', td)
    max_acce = rospy.get_param('~max_acce', max_acce)
    sphere_speed = rospy.get_param('~sphere_speed', sphere_speed)

    bridge = CvBridge()
    rospy.Subscriber("/auto_tracking", Bool, triggerCallback)
    rospy.Subscriber('/vrep/image', Image, imageCallback)
    speedPub = rospy.Publisher('/vrep/cmd_vel', Twist)
    rospy.spin()
