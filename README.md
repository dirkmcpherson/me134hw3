# me134hw3
code for letter writing robot 

The raspberry pi is set up to connect to Tufts_Wireless. You need to be able to get on Tufts_Wireless to connect to. If you can't connect to Tufts_Wireless you might have to register your device with Tufts: https://device-registration-prod-02.it.tufts.edu/login/aup

The pi should be on a static IP, connect to it by going into a terminal sshing to it. I'll post the password in our slack. 

$ssh jspi@10.245.88.70

The code is in ~/scripts/me134hw3, which is our github repo. servo_driver.py is our main file for now. It sets up and commands the servos with position commands. 