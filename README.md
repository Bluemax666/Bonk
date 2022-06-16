# Bonk
This is a program that can predict and control the trajectory of the ball in bonk.io
The program captures the screen of the game and makes a preview window with the ball trajectory and target appears on the side
Default settings are the WASD keys
Its best to put your browser window on one half your screen so there is room for the preview window next to it

To tell the program where it should capture the game and what color to track to detect the ball you can launch the bonk_setup.py script
after you launch this script you will have to put your mouse on the top corner of the game window then press P then put your mouse on the bottom opposite corner then press P 
then put your mouse over the color of the ball and press P, coordinates of the game window and the color of the ball will be automatically saved as a json file 
After that if you launch the bonk.py script it will use what is in that json file.

On the preview you can see the current ball future trajectory in magenta, the future trajectory if the up and left/right key is pressed in red, the trajectory is down right/left is pressed in blue. The target is a yellow dot if it is attainable and red if not.
You can also see the velocity vector of the ball in green and yellow arrows if the script is controlling the ball  

There is 2 modes of automatic control of the ball:
if self.tracking_mode = 'mouse' in the bonk.py script, it will try to follow the mouse (on the game window not the preview) while you press the left mouse click
if self.tracking_mode = 'numpad', you can register 10 positions by pressing : left mouse + a number key (numbers above the letters on the keyboard) and it will register that this number corresponds to your mouse position. 
Then you just have to press the number key you want to make the ball go toward the correspnding number target.  

If you install a recent version of python with anaconda, you should only need to install opencv with the command: pip install opencv-python 

video demonstration : https://www.reddit.com/r/bonkio/comments/v547ai/i_made_a_python_script_that_can_predict_and/
