#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import time

def main():
    # =============================================================
    # CONFIGURAÇÃO DA CÂMERA
    # =============================================================
    camera_id = -1 
    
    print(f"Tentando abrir a câmera {camera_id}...")
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print("ERRO: Não foi possível acessar a câmera.")
        return

    # =============================================================
    # CONFIGURAÇÃO ARUCO (ALTERADO PARA 4X4)
    # =============================================================
    # DICT_4X4_50 contém 50 variações de marcadores 4x4
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    
    # Criando o detector (API OpenCV > 4.7)
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    # =============================================================
    # CALIBRAÇÃO DA CÂMERA (Valores Reais)
    # =============================================================
    # Matriz Intrínseca (K)
    camera_matrix = np.array([
        [867.5579087775083, 0.0, 696.5006675198273],
        [0.0, 868.4321850250884, 398.84707775998703],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)

    # Coeficientes de Distorção (D): k1, k2, p1, p2, k3
    dist_coeffs = np.array([
        0.08204167161670779, -0.12296502983205114, 
        0.0038487695384758764, 0.0069080838317992265, 0.0
    ], dtype=np.float32)

    # Tamanho físico do marcador ArUco em metros
    marker_size = 0.18  # 18 cm

    print("--- LEITOR INICIADO (Dicionário 4x4) ---")
    print("Pressione CTRL+C para parar.")
    print("----------------------------------------")

    

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Aviso: Falha ao ler frame da câmera.")
                time.sleep(1)
                continue

            # Processamento em escala de cinza melhora a performance e detecção
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detectar Marcadores
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                # Estimar pose dos marcadores (posição e orientação)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, marker_size, camera_matrix, dist_coeffs
                )

                timestamp = time.strftime("%H:%M:%S")
                print(f"\n[{timestamp}] === MARCADORES DETECTADOS ===")
                
                for i, marker_id in enumerate(ids.flatten()):
                    # Calcular distância euclidiana do marcador
                    distance = np.linalg.norm(tvecs[i][0])
                    
                    # Componentes da posição (x, y, z)
                    x, y, z = tvecs[i][0]
                    
                    print(f"  ID: {marker_id}")
                    print(f"    Distância: {distance:.3f} m")
                    print(f"    Posição (x, y, z): ({x:.3f}, {y:.3f}, {z:.3f}) m")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        cap.release()
        print("Câmera liberada.")

if __name__ == "__main__":
    main()