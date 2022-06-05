# Bonk
This is a program that can predict and control the trajectory of the ball in bonk.io
The program captures the screen of the game and makes a preview window with the ball trajectory and target appears on the side
Default settings are the WASD keys
Its best to put your browser window on one half your screen so there is room for the preview window next to it

To tell the program where it should capture the game and what color to track to detect the ball you can launch the bonk_setup.py script
after you launch this script you will have to put your mouse on the top corner of the game window then press P then put your mouse on the bottom opposite corner then press P 
then put your mouse over the color of the ball and press P, coordinates of the game window and the color of the ball will be automatically saved as a json file 
After that if you launch the bonk.py script it will use what is in that json file.
