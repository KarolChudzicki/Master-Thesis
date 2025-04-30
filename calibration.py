import cv2 as cv
import numpy as np
import glob





#======================= CHESSBOARD PARAMETERS =======================#
chessboard_size = (6, 8)
square_size = 80
frame_size = (1920, 1080)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#======================= CHESSBOARD COORDINATES =======================#
chessboard_coords = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)

# Set the X and Y coordinates of the object points (Z remains 0)
chessboard_coords[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

obj_points = []  # 3D points in real world space
img_points = []  # 2D points in image plane

calib_images = glob.glob('calibration_images/captured_image*.jpg')

for image in calib_images:
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        
    ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)
        
    if ret:
        # Add object points (3D) and image points (2D) after detecting corners
        obj_points.append(chessboard_coords)
        img_points.append(corners)
            
        # Draw and display the corners on the chessboard image
        cv.drawChessboardCorners(img, chessboard_size, corners, ret)
        #cv.imshow('Chessboard', img)
        cv.waitKey(100)
    else:
        print(f"Chessboard corners not detected in {image}")
            
cv.destroyAllWindows()


ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    
with open('calib_param.txt', 'w') as file:
    file.write("===============CAMERA MATRIX===============\n")
    file.write(np.array2string(camera_matrix) + '\n')
    file.write("===============DISTANCE COEFFICIENTS===============\n")
    file.write(np.array2string(dist_coeffs) + '\n')
    file.write("===============ROTATION VECTORS===============\n")
    file.write(np.array2string(np.array(rvecs)) + '\n')
    file.write("===============TRANSLATION VECTORS===============\n")
    file.write(np.array2string(np.array(tvecs)) + '\n')
    
    
    
camera_matrix = np.array([
    [701.43142873, 0, 340.89422739],
    [0, 700.18225122, 231.27284128],
    [0, 0, 1]
])

distortion_coeffs = np.array([[-4.40734319e-01, 3.89166568e-01, -6.93778517e-05, 2.41658581e-03, -2.90304795e-01]])


rotation_vectors = np.array([
    [0.47038998, -0.23224266, -1.36597417],
    [-0.07981019, -0.16647916, 1.61505969],
    [-0.17148689, -0.48068229, 1.50307489],
    [-0.27495634, -0.34021001, 1.59257956],
    [-0.01823973, -0.27598613, 1.5211003],
    [0.31829143, -0.04453068, -1.10016429],
    [-0.21668051, -0.30041154, 0.96543265],
    [-0.26247403, 0.03388248, 1.55473485],
    [0.17765802, 0.09588896, 1.44155328],
    [0.39340981, -0.27157359, -1.39048863],
    [-0.07318423, -0.1732357, 1.48705966],
    [0.08372568, -0.47871739, 1.46270206],
    [-0.13067318, 0.17032886, 1.55662348],
    [-0.04957571, -0.2199913, 1.43057739],
    [0.04117001, -0.10908014, 0.92428516],
    [0.14109622, -0.52752174, -1.44859329],
    [0.33300622, -0.46388028, -1.1810463],
    [-0.08275248, -0.38454006, 1.49043579],
    [-0.18602322, -0.27678638, 1.10457917],
    [0.40760421, -0.06575685, -0.99286398]
])


translation_vectors = np.array([
    [[-4.16866643], [2.56145043], [23.55532557]],
    [[9.92346982], [0.47658607], [25.28933017]],
    [[-2.20838949], [-7.91495415], [29.14511795]],
    [[-1.4544579], [1.11469722], [30.59347155]],
    [[9.91888708], [-6.9861772], [26.46215193]],
    [[-6.94444007], [1.14183611], [25.40027094]],
    [[2.76274708], [-6.37237717], [25.27577111]],
    [[4.31675679], [-3.66666617], [27.87477077]],
    [[4.24090864], [-3.35521445], [30.15415769]],
    [[-5.28462817], [2.63452814], [23.51916734]],
    [[4.03681646], [-3.40591733], [26.65658219]],
    [[3.63015677], [-3.6804039], [29.34216628]],
    [[5.0664787], [-4.01336519], [18.52521328]],
    [[5.98023528], [-4.5393692], [29.46769702]],
    [[2.61681378], [-5.3257859], [29.71997933]],
    [[-4.5112023], [2.47364792], [29.94940236]],
    [[-3.71141982], [1.23030042], [23.98786941]],
    [[5.53142058], [-7.20677587], [26.92446391]],
    [[2.56458315], [-5.54783017], [27.04624868]],
    [[-8.02427971], [0.51851539], [25.71631894]]
])