1. Create project folder:

`git clone https://github.com/EngSaleha/EE496_Project.git`

3. Create Virtual Environment inside the folder:

`cd EE496_Project`

`python3 -m venv env`

5. Activate it:
   
`. env/Scripts/\activate` 

7. Install the required libraries:
```
sudo pip3 install -U pip
sudo apt-get install -y i2c-tools
sudo pip3 install adafruit-pca9685sudo 
sudo pip3 install rpi_ws281x
sudo pip3 install numpy
```

9. To track changes:
    
`git pull`

11. To update the repo:
```
git add .
git commit -m "Comment Your Changes"
git push -u origin main
```
