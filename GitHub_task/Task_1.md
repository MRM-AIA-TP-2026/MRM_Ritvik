# Task 1 

## Installing and configuring git 
 i used the command ```sudo apt install git-all``` to first install git. Then used ```git config --global user.name ""``` and ```git config --global user.email""``` to add the email. 
Then I generated an ssh key using ```ssh-keygen -t ed25519``` and used ```cat ~/.ssh/id_ed25519.pub``` to show the ssh key which I added in github.
```git clone <repo ssh>``` used this for cloning the repo.

```mkdir GitHub_task``` to create a folder and ```cd``` to change into that folder. ```sudo apt intall --classic code``` to install code and ```code hello.py``` to create the file and code in it.
```git add hello.py``` to add the file to the stage and ```git commit -m "added hello.py"``` to commit the chnages and then used ```git push``` to push the changes.
