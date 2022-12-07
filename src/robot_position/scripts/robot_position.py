import rospy
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

room = 'X'
lastUpdate = 0

WINDOW_NAME = 'Robot Position'
BORDER_Y = 4.8
BORDER_XAB = 3.7
BORDER_XBC = 6.9

def posCallback(pos):
    global room
    global lastUpdate
    lastUpdate = rospy.get_time()
    if pos.pose.position.y > BORDER_Y:
        room = 'D'
    elif pos.pose.position.x < BORDER_XAB:
        room = 'A'
    elif pos.pose.position.x < BORDER_XBC:
        room = 'B'
    else:
        room = 'C'
    # rospy.loginfo(f'current pos is {pos.pose.position.x}, {pos.pose.position.y}')
    
if __name__ == '__main__':
    rospy.init_node('robot_position')
    rospy.Subscriber("/slam_out_pose", PoseStamped, posCallback)
    img = np.zeros([256,256,3],dtype=np.uint8)
    img.fill(255)
    cv2.putText(img, 'Current Room:', (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
    cv2.imshow(WINDOW_NAME, img)
    cv2.waitKey(10)

    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (rospy.get_time() - lastUpdate) > 2:
                room = 'X'
            display = img.copy()
            cv2.putText(display, room, (70, 210), cv2.FONT_HERSHEY_SIMPLEX, 7, (0,0,0), 5)
            cv2.waitKey(10)
            cv2.imshow(WINDOW_NAME, display)
            rate.sleep()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
