#!/usr/bin/env python3
"""
Script para gerar marcadores ArUco para simulação Gazebo
"""
import cv2
import cv2.aruco as aruco
import numpy as np

# Configurações
marker_size = 400  # pixels (400x400 para boa resolução)
marker_ids = [0, 1, 2, 3]  # IDs dos marcadores (um para cada ponta do arco)
output_dir = "/root/internship_ws/src/avfl_autonomy/drone_gazebo/models/aruco_marker/materials/textures"

# Dicionário ArUco (6x6 com 250 marcadores)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Gerar os marcadores
for marker_id in marker_ids:
    # Gerar imagem do marcador
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    
    # Adicionar borda branca ao redor do marcador para melhor detecção
    border_size = 50
    bordered_image = cv2.copyMakeBorder(
        marker_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )
    
    # Salvar imagem
    output_path = f"{output_dir}/marker_{marker_id}.png"
    cv2.imwrite(output_path, bordered_image)
    print(f"Marcador {marker_id} gerado: {output_path}")

print(f"\nTotal de {len(marker_ids)} marcadores gerados com sucesso!")
print(f"Tamanho do marcador: {marker_size}x{marker_size} pixels")
print(f"Tamanho com borda: {marker_size + 2*border_size}x{marker_size + 2*border_size} pixels")
