import cv2
import mediapipe as mp
import math




class DetectHand:
    def __init__(self,mode = False,min_detection_confidence=0.5, min_tracking_confidence=0.5, max_hands = 2):
        self.mode = mode
        self.right_clicked = 0
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence  = min_tracking_confidence
        self.max_hands                = max_hands
        self.landmark_list            = list()
        self.finger_tips              = list()
        self.mode                     = 0
        self.mp_drawing               = mp.solutions.drawing_utils
        self.mp_hands                 = mp.solutions.hands
        self.hands                    = self.mp_hands.Hands(
                                                            min_detection_confidence=self.min_detection_confidence,
                                                            min_tracking_confidence=self.min_tracking_confidence
                                                            )

        self.line_length = -1
    def detect_hands(self,image, draw = 0):

            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            self.result = self.hands.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if self.result.multi_hand_landmarks:
                for hand_landmarks in self.result.multi_hand_landmarks:
                    if draw:
                        self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                        '''fps = 1/(time.time() - ptime)

                        cv2.putText(image, f'fps : {int(fps)}', (30,30), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0 , 0), 2)'''
            return image
    def find_distance(self,point1, point2):
        return math.sqrt( (point2[1] - point1[1])**2 + (point2[0] - point1[0])**2 )

    def mark_points(self, image, points):
        for point in points:
            cv2.circle(image, (point[1],point[2]),8, (255, 0, 0), -1)

    def get_coordinates(self,image, hand = 0, draw = 0, point1 = 4, point2 = 8):

        h, w, n = image.shape
        self.landmark_list.clear()
        self.finger_tips.clear()
        if self.result.multi_hand_landmarks:
            for idx, coordinate in enumerate( self.result.multi_hand_landmarks[hand].landmark):
                x = int(coordinate.x * w)
                y = int(coordinate.y * h)
                self.landmark_list.append( [idx, x, y])
                if(idx in [4, 8, 12,16, 20]):
                   self.finger_tips.append([idx, x, y])
        if(draw == 1 and len( self.landmark_list) > 0):

            coordinate_1 = ( self.landmark_list[point1][1],  self.landmark_list[point1][2])
            coordinate_2 = ( self.landmark_list[point2][1],  self.landmark_list[point2][2])
            cx           = (coordinate_1[0] + coordinate_2[0])//2
            cy           = (coordinate_1[1] + coordinate_2[1])//2
            self.line_length = self.get_distance(coordinate_1, coordinate_2)
            self.mark_points(image, coordinate_1, cooridnate_2)
            if(self.line_length < 80):
                cv2.circle(image, (cx, cy), 8, (0, 255, 0), -1)
            if(self.line_length > 80):
                cv2.circle(image, (cx, cy), 8, (255, 2, 0), -1)
        return self.line_length

    def fingers_up(self, image):
        fingers = list()
        if(len(self.finger_tips) > 1 ):
            thumb_level = self.finger_tips[0][2]
            for finger in self.finger_tips[1:]:
                if(abs(finger[2] - thumb_level) > 70):
                    fingers.append(finger)

        return fingers

def main():

    cap = cv2.VideoCapture(0)
    detector = DetectHand(min_detection_confidence = 0.7)
    while(cap.isOpened()):
        success, frame = cap.read()
        image = detector.detect_hands(frame, draw = 1)
        detector.get_coordinates(image, draw = 1)
        cv2.imshow("hands", image)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()


if __name__ == "__main__":
    main()
