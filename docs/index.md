# Welcome to the Sapienza Technology Team Documentation!

Documentation is the lifeline of our Rover. Whether you are working on the robotic arm, the drone, or the science mission, documenting your work allows the team to grow and prevents knowledge loss.


This guide will walk you through setting up your computer to write, preview, and publish documentation using **Zensical** and **GitHub**.

Please follow this guide to avoid getting **salsicciato** by us maintainers.

![Salsicciato](assets\images\salsiccia.webp)

If you have troubles committing to the repo please talk to your group leaders first. We can update and commit on your behalf if you don't want to learn Markdown.

## 0. What are Github and Zensical?
**Github** is just a way for us to have backup of what is done, allowing everyone to edit the documentation and allowing us to revert it to the previous state if, let's say, someone deletes a page or does something by mistake.  
We found no suitable video tutorial to link, so we will make one and publish it on youtube asap.

**Zensical** is a toolchain to build static sites using a language called *Markdown*. It's very easy to use and you find some examples on [this page](https://sapienza-technology.github.io/documentation/markdown/).

---

## 1. Prerequisites & Installation

Before writing, you need to set up your "Development Environment". You only need to do this once.

### Step A: Install the Tools
1.  **VS Code**: Download and install [Visual Studio Code](https://code.visualstudio.com/). This is the editor we use.
2.  **Git**: Download and install [Git](https://git-scm.com/downloads).
       *Windows users:* Select "Use Git from the Windows Command Prompt" during installation.
3.  **Python**: Zensical runs on Python. Download [Python 3.x](https://www.python.org/downloads/).
       !!! warning "Important for Windows"
        During installation, make sure to check the box **"Add Python to PATH"** at the bottom of the installer window.
4. **PyEnv**: it is recommended by the developers to install Zensical on a virtual environment.
       If you are on Ubuntu (as you should) install pyenv using the official installer:

    `curl https://pyenv.run | bash`

    Then follow the message that appears in the console and add those lines to ~/.bashrc


???+ info "About PyEnv installation"
       You can skip PyEnv and install Zensical out of the virtual environment and it will work smoothly. 
       Installing it in a venv is just how the developers recommend to install it.

### Step B: VS Code Setup
Open VS Code and install these recommended extensions (click the square icon on the left sidebar):

*   **Markdown All in One** (by Yu Zhang) - Helps with shortcuts and table of contents.

*   **Code Spell Checker** (by Street Side Software) - Typos are unprofessional!
*   **GitHub Pull Requests** - Makes syncing easier.

### Step C: Install Zensical
Open up a terminal window and install Zensical by first setting up a virtual environment and then using pip to install the Zensical package into it:

```
python3 -m venv .venv
source .venv/bin/activate
pip install zensical
```

???+ info "About PyEnv installation"
       You can skip PyEnv and install Zensical out of the virtual environment and it will work smoothly. 
       Installing it in a venv is just how the developers recommend to install it.

### Step D: Clone repo
You are now ready to clone the repo 'documentation' from the STT github and start editing it.

Every time you open the project to do some changes first **Pull** to see if there are any changes incoming.

---

## 2. How to contribute
In the terminal you have activated your virtual environment on just run:

* `zensical build` to build the project. But you can simple run the `serve` command and it will automatically build at every save.

* `zensical serve` to start the web server to see your live edits on the code.

* Go to [localhost:8000](http://localhost:8000/documentation) or simply click this link.

* Verify your changes and then **Commit** your edits. When you are done editing just **Push** in the repo.

!!! note "About versioning"
       As a good practice always **Push** your changes when you finish your edit session.  
       Remember to only push changes after checking that the local version is tested, otherwise you could get **salsicciato** if you have to correct your pushes.  
       Please write good commit comments, so we can know what you just uploaded(another way to get **salsicciato**).

It is recommended to read the [Markdown page](https://sapienza-technology.github.io/documentation/markdown/) to understand how to write markdown.

---