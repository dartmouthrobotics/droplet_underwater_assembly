# Minimal control experiment node

# First pass is only for the yaw axis

Try to center the tag in the yaw axis

What does that look like and what data to report?

We want to take the atan2 of the tag reading in each frame and react with a change in the motor speed

So in each frame, we take the xy position of the tag in the base link frame
We can plot the error over time and run the code for a specified amount of seconds (a cli arg)

binary pwm python file
