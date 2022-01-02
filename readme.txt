For more information, please take a look at "assignment 1.pdf"

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Library used: Eigen, glut
https://eigen.tuxfamily.org/index.php?title=Main_Page
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Target Platform: Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls of the application:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Make sure caps lock is disabled
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

"b" to select a bvh file
"a" to save the current animation file to bvh
"i" to print all joint name and id in the console

arrow up/down: control the target position in the y-axis
arrow left/right: control the target position in the x-axis
"[" and "]": control the target position in the z-axis

"," and ".": select the current joint through the joint tree structure

"f" to perform IK using the selected joint and target values once.
    If you hold the f button, it perform IK repeatedly and eventually reach the target position
"v" to perform IK using the selected joint and target values to iteration the final values with a maximum step of 1000
    and error value of 0.03
"t" to perform IK on left and right hand to the target position(2, 1.5, -0.128156) and (-2, 1.5, -0.128156) respectivitely
    If you hold the t button, it perform IK repeatedly and eventually reach the target position
