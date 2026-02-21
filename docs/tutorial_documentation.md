# How to Contribute to the Documentation

Documentation is the lifeline of our Rover. Whether you are working on the robotic arm, the drone, or the science mission, documenting your work allows the team to grow and prevents knowledge loss.

This guide will walk you through setting up your computer to write, preview, and publish documentation using **Zensical** and **GitHub**.

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
       Install pyenv using the official installer:

    `curl https://pyenv.run | bash`

    Add these lines to ~/.bashrc:
    ```
    export PYENV_ROOT="$HOME/.pyenv"
    export PATH="$PYENV_ROOT/bin:$PATH"
    eval "$(pyenv init --path)"
    eval "$(pyenv init -)"
    ```


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

### Step D: Clone repo
Clone the repo 'documentation' from the STT github.

## 2. How to contribute
In the terminal you have activated your virtual environment on just run:

* `zensical build` to build the project.

* `zensical serve` to start the web server to see your live edits on the code.

* Go to [localhost:8000](http://localhost:8000/documentation) or simply click this link.

Remember to only push changes after checking that the local version is tested.
Please write good commit comments, so we can know what you just uploaded.

---