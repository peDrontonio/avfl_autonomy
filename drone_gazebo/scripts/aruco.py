import cv2
import numpy as np
import math


MARKER_SIZE = 0.205      # 20.5 cm
GAP_X = 1.5 - MARKER_SIZE            
GAP_Y = 1.5 - MARKER_SIZE            

# Camera Calibration (Replace with your actual calibration for best results)
FOCAL_LENGTH = 1100
CX, CY = 640, 360
CAMERA_MATRIX = np.array([[FOCAL_LENGTH, 0, CX], [0, FOCAL_LENGTH, CY], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1)) 

# Calculated Offsets
dx = (GAP_X / 2) + (MARKER_SIZE / 2)
dy = (GAP_Y / 2) + (MARKER_SIZE / 2)


def get_marker_corners(center_x, center_y, size):
    s = size / 2
    return np.array([
        [center_x - s, center_y - s, 0], # Top-Left
        [center_x + s, center_y - s, 0], # Top-Right
        [center_x + s, center_y + s, 0], # Bot-Right
        [center_x - s, center_y + s, 0]  # Bot-Left
    ], dtype=np.float32)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.degrees(x), np.degrees(y), np.degrees(z)

# ID 2 is Top Left  | ID 3 is Top Right
# ID 0 is Bottom Left  | ID 1 is Bottom Right

OBJ_POINTS_MAP = {
    2: get_marker_corners(-dx, -dy, MARKER_SIZE), # Top Left
    3: get_marker_corners(+dx, -dy, MARKER_SIZE), # Top Right
    1: get_marker_corners(+dx, +dy, MARKER_SIZE), # Bot Right
    0: get_marker_corners(-dx, +dy, MARKER_SIZE)  # Bot Left
}

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()



def get_drone_position(frame):
  
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    drone_pos = (0, 0, 0, 0, 0, 0)
    command = "HOVER"
    found = False
    
    # Deadzone for command logic (e.g. 10cm)
    DEADZONE = 0.10 

    if ids is not None:
        all_img_pts = []
        all_obj_pts = []
        ids_seen = [id[0] for id in ids]
        
        for i in range(len(ids)):
            current_id = ids[i][0]
            if current_id in OBJ_POINTS_MAP:
                all_img_pts.extend(corners[i][0])
                all_obj_pts.extend(OBJ_POINTS_MAP[current_id])

        # Need at least 1 marker (4 corners) to solve
        if len(all_obj_pts) >= 4:
            img_pts_np = np.array(all_img_pts, dtype=np.float32)
            obj_pts_np = np.array(all_obj_pts, dtype=np.float32)

            # Solve PnP
            success, rvec, tvec = cv2.solvePnP(obj_pts_np, img_pts_np, CAMERA_MATRIX, DIST_COEFFS)

            if success:
                # 1. Calculate Position (Translation)
                R, _ = cv2.Rodrigues(rvec)
                R_inv = np.transpose(R) # Inverse rotation (Camera relative to World)
                cam_pos_world = -np.dot(R_inv, tvec)

                c_x = cam_pos_world[0][0]
                c_y = cam_pos_world[1][0]
                c_z = cam_pos_world[2][0]
                
                # 2. Calculate Orientation (Rotation)
                # We use R_inv because we want the Drone's angle relative to the floor
                pitch_x, yaw_y, roll_z = rotationMatrixToEulerAngles(R_inv)
                
                # Return tuple: X, Y, Z, Pitch, Yaw, Roll
                drone_pos = (c_x, c_y, c_z, pitch_x, yaw_y, roll_z)
                found = True

                # 3. Generate Commands
                cmd_x = ""
                cmd_y = ""

                # Horizontal (X)
                if c_x < -DEADZONE:  cmd_x = "MOVE RIGHT >>"
                elif c_x > DEADZONE: cmd_x = "<< MOVE LEFT"
                else:                cmd_x = "[CENTER X]"

                # Vertical (Y)
                if c_y < -DEADZONE:  cmd_y = "MOVE DOWN v"
                elif c_y > DEADZONE: cmd_y = "MOVE UP ^"
                else:                cmd_y = "[CENTER Y]"

                command = f"{cmd_x} | {cmd_y}"

                # 4. Drawing & OSD
                cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, 0.1)
                
                # Info Box Background
                cv2.rectangle(frame, (10, 10), (450, 180), (0,0,0), -1)
                
                # Data Text
                cv2.putText(frame, f"POS: X:{c_x:.2f} Y:{c_y:.2f} Z:{c_z:.2f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"ROT: P:{pitch_x:.1f} Y:{yaw_y:.1f} R:{roll_z:.1f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)
                cv2.putText(frame, f"CMD: {command}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                id_str = ",".join(str(x) for x in ids_seen)
                cv2.putText(frame, f"Locked IDs: {id_str}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return drone_pos, command, frame, found


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)

    print("Starting Drone Tracking...")

    while True:
        ret, frame = cap.read()
        if not ret: break
        
        # Unpack the new 6-value tuple
        pos_data, cmd, out_frame, is_found = get_drone_position(frame)
        
        if is_found:
            x, y, z, pitch, yaw, roll = pos_data
            
            # Print for debugging
            # Note: Pitch/Yaw/Roll mapping might vary depending on how your camera is mounted
            print(f"POS: [{x:.2f}, {y:.2f}, {z:.2f}] | ROT: [P:{pitch:.1f}, Y:{yaw:.1f}, R:{roll:.1f}] | {cmd}")
        
        cv2.imshow("Drone Guidance", out_frame)
        if cv2.waitKey(1) == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()