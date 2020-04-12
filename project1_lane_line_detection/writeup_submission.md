# solutions

Firstly, I converted an given image to grayscale, with the column and row sizes of the image.

To select the region of my interest, I used the helper function from the template.

[image1]: .CarND-LaneLines-P1//Test/grayscale.jpg 'Grayscale'

To draw two lines - the left line and right line, I appended the values of x and y coordinates from Hough space. The boundary centers around the middle of the x coordinate of the given image.

Calculating the gradients between x and y endpoints of each line, I identified it as a left line when the value is negative, otherwise right. 

Each segment line varies, so does its gradient value. I took the mean value of the gradient values.

To extend the detected lane lines to the edge of the given image, I calculated the intersection of x coordinate between the extended line and the bottom edge of the image.

Afterwards, I drew the left and right lines on the blank image.

[image2]: ./Test/result.jpg 'result'

In the end, I compose the image with the original image.

# reflections

Shortcomings would be as follows.

When driving on a wide road, the endpoints of the lines can overshoot the edges - so my y coordinate, ysize, won't be valid. 
Also when far lane lines curve greatly, this would influence on the mean value of the lane line and possibly, the predicted lines could bend irregularly. When there's some big vehicle travelling ahead nearby, this would also hinder the observable lines therefore could worsen the result.

Improvements would be as follows.
Turning parameters could help.
Also, remembering the trajectory of the lanelines would help when some vehicle interrupts ahead. 

