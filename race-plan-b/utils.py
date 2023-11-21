import cv2	
import numpy as np

def make_poly(width,height,i = 75, j = -10, k = 30 ):

    return np.array([
    [
        (0, height - k),
        (int(width / 2) - i , int(height / 2) - j -20),
        (int(width / 2) + i , int(height / 2) - j),
        (width, height -k)
    ]
    ])

def make_hood_poly(width,height,i = 15, j = 0, k = 0 ):

	return np.array([
	[
		(110, height - k),
		(180 , 400),
		(340 ,400),
		(425, height -k)
	]
	])

def translate(inp_min, inp_max, out_min, out_max,inp_error):
	return (float(inp_error - inp_min) * float(out_max-out_min) / float(inp_max - inp_min) + float(out_min))

def draw_lines(image, lines, color):
	#line_image = image.copy()
	for line in lines:
	# print(lines[0])
		x1, y1, x2, y2 = line.reshape(4) 
		# print(type(x1))
		cv2.line(image, (x1, y1) ,(x2, y2), color, 10)			
	return image

def make_coordinates(image, line):
	if(isinstance(line, np.ndarray)):
		slope, intercept = line
		y1 = image.shape[0]
		y2 = int(y1 * (3/5))
		x1 = int((y1 - intercept) / slope)
		x2 = int((y2 - intercept) / slope)
		return np.array([x1, y1, x2, y2])
	else:
		return np.array([0,0,0,0])

def calc_avg_line(image, lines):
	left_lines = []
	right_lines = []
	
	for line in lines:
		x1, y1, x2, y2 = line.reshape(4) 
		parameters = np.polyfit((x1,x2), (y1,y2), 1)
		slope = parameters[0]
		intercept = parameters[1]
		if(slope <= 0):
			if(slope != 0):
				left_lines.append((slope, intercept))
				# left_lines.append((slope, intercept))
		else:
			right_lines.append((slope, intercept))

	left_line_avg = np.average(left_lines, axis=0) 
	right_line_avg = np.average(right_lines, axis=0) 


	left_line = make_coordinates(image, left_line_avg)
	right_line = make_coordinates(image, right_line_avg)
	
	left_line_avg_fix = left_line_avg if (isinstance(left_line_avg, np.ndarray)) else np.array([0, 0]) 	

	
	right_line_avg_fix = right_line_avg if (isinstance(right_line_avg, np.ndarray)) else np.array([0, 0]) 	
	
	error = (left_line_avg_fix[0] + right_line_avg_fix[0]) / 2
		
 
	return np.array([left_line, right_line]), error , right_line_avg_fix[0] , left_line_avg_fix[0]
