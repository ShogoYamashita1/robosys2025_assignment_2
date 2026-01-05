#!/usr/bin/env python3

import cv2
import mediapipe as mp
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


class Talker():
    def __init__(self, nh):
        self.pub = nh.create_publisher(Int16MultiArray, "hand_result", 10)

    def publish(self, state):
        msg = Int16MultiArray()
        msg.data = state
        self.pub.publish(msg)

def hand_process(cap, hands):
# For webcam input:
      #while cap.isOpened():
      if cap.isOpened():
        success, image = cap.read()
        if not success:
          print("Ignoring empty camera frame.")
          # If loading a video, use 'break' instead of 'continue'.
          #continue
          return 0, False

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image = cv2.flip(image, 1)# 画像を左右反転
        results = hands.process(image)# 推論

        height, width, _ = image.shape
        close_state = [0]*5
        B = [0]*5
        G = [255]*5
        result = 0

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
          for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style()
            )

            handnum00 = hand_landmarks.landmark[0]
            handnum04 = hand_landmarks.landmark[4]
            handnum08 = hand_landmarks.landmark[8]
            handnum09 = hand_landmarks.landmark[9]
            handnum12 = hand_landmarks.landmark[12]
            handnum16 = hand_landmarks.landmark[16]
            handnum20 = hand_landmarks.landmark[20]
            x00 = handnum00.x
            y00 = handnum00.y
            x04 = handnum04.x
            y04 = handnum04.y
            x08 = handnum08.x
            y08 = handnum08.y
            x09 = handnum09.x
            y09 = handnum09.y
            x12 = handnum12.x
            y12 = handnum12.y
            x16 = handnum16.x
            y16 = handnum16.y
            x20 = handnum20.x
            y20 = handnum20.y
            X00 = int(x00 * width)
            Y00 = int(y00 * height)
            X04 = int(x04 * width)
            Y04 = int(y04 * height)
            X08 = int(x08 * width)
            Y08 = int(y08 * height)
            X09 = int(x09 * width)
            Y09 = int(y09 * height)
            X12 = int(x12 * width)
            Y12 = int(y12 * height)
            X16 = int(x16 * width)
            Y16 = int(y16 * height)
            X20 = int(x20 * width)
            Y20 = int(y20 * height)

            thumb  = (math.sqrt( (X04 - X00)**2 + (Y04 - Y00)**2 ))
            index  = (math.sqrt( (X08 - X00)**2 + (Y08 - Y00)**2 ))
            middle = (math.sqrt( (X12 - X00)**2 + (Y12 - Y00)**2 ))
            ring   = (math.sqrt( (X16 - X00)**2 + (Y16 - Y00)**2 ))
            pinky  = (math.sqrt( (X20 - X00)**2 + (Y20 - Y00)**2 ))
            palm   = (math.sqrt( (X09 - X00)**2 + (Y09 - Y00)**2 ))

            fingers = [thumb, index, middle, ring, pinky]

            for fin, i in zip(fingers, range(5)):
                if(fin < palm):
                    close_state[i] = 1
                    B[i] = 255
                    G[i] = 0
                else:
                    close_state[i] = 0
                    B[i] = 0
                    G[i] = 255

            if (all(i == 0 for i in close_state)):
                result = 1# パー
                cv2.putText(image, "pha", (0, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            elif (all(i == 1 for i in close_state)):
                result = 2# グー
                cv2.putText(image, "ghu", (0, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            elif (close_state == [1, 0, 0, 1, 1]):
                result = 3#チョキ
                cv2.putText(image, "choki", (0, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            else:
                result = 0#none
                cv2.putText(image, "none", (0, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)


            cv2.putText(image, f"THUMP({thumb:.0f})",   (0,  30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (B[0], G[0], 0), 2)
            cv2.putText(image, f"INDEX({index:.0f})",   (0,  60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (B[1], G[1], 0), 2)
            cv2.putText(image, f"MIDDLE({middle:.0f})", (0,  90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (B[2], G[2], 0), 2)
            cv2.putText(image, f"RING({ring:.0f})",     (0, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (B[3], G[3], 0), 2)
            cv2.putText(image, f"PINKY({pinky:.0f})",   (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (B[4], G[4], 0), 2)
            cv2.putText(image, f"PALM({palm:.0f})",     (0, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            #cv2.putText(image, f"RESULT({result})",        (0, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Hands', image)
        if cv2.waitKey(5) & 0xFF == 27:
          #break
          return True

        return close_state, False


def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    hands = mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    rclpy.init()

    node = Node("talker")
    talker = Talker(node)
    while rclpy.ok():
        state, flag = hand_process(cap, hands)
        talker.publish(state)
        rclpy.spin_once(node, timeout_sec=0.001)
        if flag:
            break

    cap.release()


if __name__ == "__main__":
    main()
