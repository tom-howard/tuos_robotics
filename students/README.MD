# Automated installer for COM2009 and COM3528 ROS environment

## Description

This part of the repo is for COM2009 and COM3528 students (and anyone else interested), who wishes to install the Robotics Teaching environment on their own computer running Ubuntu 20.04, natively or via WSL.

## Usage

Download and run the automated script `full_install.sh`, which *should* set everything up for you.

```bash
wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/full_install.sh
chmod +x full_install.sh && ./full_install.sh
```

### MDK only

To install/update the MDK only, use the `update_mdk.sh`. Note that this does not install MDK dependencies, nor does it install robot switching scripts.

>**Note:** COM3528 students, you should use the main script in the first instance, and only use the this one if you're MDK installation somehow ended up broken  

```bash
wget https://raw.githubusercontent.com/tom-howard/tuos_robotics/main/students/update_mdk.sh
chmod +x update_mdk.sh && ./update_mdk.sh
```

>**Note:** *Keep an eye on terminal output, as there will be a few prompts that will require your input.*
