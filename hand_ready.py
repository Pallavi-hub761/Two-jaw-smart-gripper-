import cv2
import mediapipe as mp
import numpy as np
from collections import deque

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5, max_num_hands=2)
mp_drawing = mp.solutions.drawing_utils

# Motion tracking
POSITION_HISTORY = deque(maxlen=5)
MOTION_THRESHOLD = 5.0

def calculate_angles(landmarks):
    """Check if fingers are open (angles > 160°) including thumb"""
    # Finger angles (index to pinky)
    finger_tips = [mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                   mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.PINKY_TIP]
    mcp_joints = [mp_hands.HandLandmark.INDEX_FINGER_MCP, mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
                  mp_hands.HandLandmark.RING_FINGER_MCP, mp_hands.HandLandmark.PINKY_MCP]
    
    angles = []
    for tip, mcp in zip(finger_tips, mcp_joints):
        pip = landmarks[mcp + 1]
        vec1 = np.array([landmarks[mcp].x - pip.x, landmarks[mcp].y - pip.y])
        vec2 = np.array([landmarks[tip].x - pip.x, landmarks[tip].y - pip.y])
        cos_angle = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
        angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
        angles.append(angle)
    
    # Thumb angle (special handling)
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = landmarks[mp_hands.HandLandmark.THUMB_IP]
    thumb_mcp = landmarks[mp_hands.HandLandmark.THUMB_MCP]
    vec1 = np.array([thumb_mcp.x - thumb_ip.x, thumb_mcp.y - thumb_ip.y])
    vec2 = np.array([thumb_tip.x - thumb_ip.x, thumb_tip.y - thumb_ip.y])
    thumb_angle = np.degrees(np.arccos(np.clip(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)), -1.0, 1.0)))
    
    # All fingers (including thumb) must be open
    return np.mean(angles) > 160 and thumb_angle > 160

def is_facing_camera(landmarks):
    """Check if fingertips are closer to camera than wrist"""
    wrist_z = landmarks[mp_hands.HandLandmark.WRIST].z
    tip_z = np.mean([landmarks[tip].z for tip in [
        mp_hands.HandLandmark.INDEX_FINGER_TIP, 
        mp_hands.HandLandmark.MIDDLE_FINGER_TIP
    ]])
    return tip_z < wrist_z

def is_low_motion(landmarks, frame):
    """Check if hand moved < 5px compared to history"""
    if len(POSITION_HISTORY) == 0:
        return True
    wrist_pos = (landmarks[mp_hands.HandLandmark.WRIST].x * frame.shape[1], 
                 landmarks[mp_hands.HandLandmark.WRIST].y * frame.shape[0])
    avg_movement = np.mean([np.linalg.norm(np.array(wrist_pos) - np.array(pos)) for pos in POSITION_HISTORY])
    POSITION_HISTORY.append(wrist_pos)
    return avg_movement < MOTION_THRESHOLD

def check_no_curling(landmarks):
    """Check if fingers aren't curled (thumb-to-index distance)"""
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    curl_distance = np.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
    return curl_distance > 0.1

def get_hand_orientation(landmarks):
    """Determine if hand is left or right and palm direction"""
    # Vector from wrist to middle MCP
    wrist = landmarks[mp_hands.HandLandmark.WRIST]
    middle_mcp = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
    direction_vector = np.array([middle_mcp.x - wrist.x, middle_mcp.y - wrist.y])
    
    # Check thumb position relative to pinky
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    pinky_tip = landmarks[mp_hands.HandLandmark.PINKY_TIP]
    is_right_hand = thumb_tip.x < pinky_tip.x
    
    # For both hands, palm faces camera when direction vector points upward in image coordinates
    palm_facing = direction_vector[1] < 0  # Negative y-direction in image coords
    
    return is_right_hand, palm_facing

def is_hand_ready(landmarks, frame):
    """Main readiness check with hand orientation"""
    is_right_hand, palm_facing = get_hand_orientation(landmarks)
    return (calculate_angles(landmarks) and
            is_facing_camera(landmarks) and
            is_low_motion(landmarks, frame) and
            check_no_curling(landmarks) and
            palm_facing)

# Main loop
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    if results.multi_hand_landmarks:
        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            # Draw landmarks
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Check readiness
            ready = is_hand_ready(hand_landmarks.landmark, frame)
            hand_type = handedness.classification[0].label
            status = f"{hand_type} READY" if ready else f"{hand_type} NOT READY"
            color = (0, 255, 0) if ready else (0, 0, 255)
            cv2.putText(frame, status, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
    
    cv2.imshow('Hand Detection', frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()