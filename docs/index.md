# Welcome to the Sapienza Technology Team Documentation!

Documentation is the lifeline of our Rover. Whether you are working on the robotic arm, the drone, or the science mission, documenting your work allows the team to grow and prevents knowledge loss.


This guide will walk you through setting up your computer to write, preview, and publish documentation using **Zensical** and **GitHub**.

Please follow this guide to avoid getting **salsicciato** by us maintainers.

![Salsicciato](assets\images\salsiccia.webp)

If you have troubles committing to the repo please talk to your group leaders first. We can update and commit on your behalf if you don't want to learn Markdown.

## What are Github and Zensical?
**Github** is just a way for us to have backup of what is done, allowing everyone to edit the documentation and allowing us to revert it to the previous state if, let's say, someone deletes a page or does something by mistake.  
We found no suitable video tutorial to link, so we will make one and publish it on youtube asap.

**Zensical** is a toolchain to build static sites using a language called *Markdown*. It's very easy to use and you find some examples on [this page](https://sapienza-technology.github.io/documentation/markdown/).

---

## 1. Prerequisites & Installation

Before writing, you need to set up your "Development Environment". You only need to do this once.

### Step A: Install the Tools
1.  **VS Code**: Download and install [Visual Studio Code](https://code.visualstudio.com/). This is the editor we use.
2.  **Git**: Download and install [Git](https://git-scm.com/downloads).  
3.  **Python**: Zensical runs on Python. Download [Python 3.x](https://www.python.org/downloads/) or simply download it through the **"Extensions"** panel of VSCode.

!!! warning "Important for Windows"

       * During Git installation, select **"Use Git from the Windows Command Prompt"**.

???+ info "PyEnv"
       It is recommended by the developers to install Zensical on a virtual environment.  
       
       If you want to use PyEnv, during Python installation, make sure to check the box **"Add Python to PATH"** at the bottom of the installer window.
      
       If you are on Ubuntu (as you should) install pyenv using the official installer:  
       `curl https://pyenv.run | bash`  
       Then follow the message that appears in the console and add those lines to ~/.bashrc

       You can skip PyEnv and install Zensical out of the virtual environment and it will work smoothly. 
       Installing it in a venv is just how the developers recommend to install it, but we don't care, it's simpler without it.

### Step B: VS Code Setup
Open VS Code and install these recommended extensions (click the square icon on the left sidebar):

*   **Markdown All in One** (by Yu Zhang) - Helps with shortcuts and table of contents.
*   **Code Spell Checker** (by Street Side Software) - Typos are unprofessional!

### Step C: Install Zensical
Open up a terminal window and install Zensical by using pip to install the Zensical package:

```
pip install zensical
```

???+ info "PyEnv"
       If you are using PyEnv create and activate your environment before installing zensical using:  
       
       ```
       python3 -m venv .venv
       source .venv/bin/activate
       ```

### Step D: Clone repo
You are now ready to clone the repo 'documentation' from the STT github and start editing it.

Every time you open the project to do some changes first **Pull** to see if there are any changes incoming.  
If GitHub signals any conflict please report to us.

---

## 2. How to build and commit
In the terminal you have activated your virtual environment on just run:

* `zensical build` to build the project. But you can simple run the `serve` command and it will automatically build at every save.

* `zensical serve` to start the web server to see your live edits on the code.

* Go to [localhost:8000](http://localhost:8000/documentation) or simply click this link.

It is recommended to read the [Markdown page](https://sapienza-technology.github.io/documentation/markdown/) to understand how to write markdown.

## 3. How to actually use Github

Look the guide [here](https://sapienza-technology.github.io/documentation/example/) on how to edit your files and commit on the repo.

---