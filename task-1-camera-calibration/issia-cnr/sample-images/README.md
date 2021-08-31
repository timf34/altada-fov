# Contents

- This directory contains images of a football pitch from the ISSIA CNR soccer dataset 
  - Each image is from a different location/ angle and has its pitch markings/ points named (ie Fisso 20)
<img src="Reference-Camera-1.bmp" width="50%"> </p>
  
- We include a Python file with numpy arrays filled out with the calibration imfo mapping pixels to real world coordinates.
  - Note that the arrays for cameras 5 and 6 have considerably more points filled 
  - We used these for calculating a homography matrix mapping pixels to real world coordinates


- This directory also contains a PDF with the calibration data laid out in a table, including: 
  - The mapping between pixels and real world points
  - The camera rotation matrices and approximate real world positiosn (although these are estimates!)
