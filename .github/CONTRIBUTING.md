# Contributing Guidelines

Welcome to the team! To maintain an organized code workspace and a clean Git history, all collaborators must follow these exact repository workflow rules.

---

## 1. Where to write code

ALL of the team code should go in this folder:

TeamCode &rarr; src &rarr; main &rarr; java &rarr; org.firstinspires.ftc.teamcode

NEVER edit any of the source files from the FtcRobotController folder or anything outside the TeamCode folder, as those are written by FIRST and editing them will cause the project to break.

## 2. Do not update the Gradle or anything else related to the SDK
If Android Studio or VSCode asks you to update Gradle, migrate to Gradle Daemon, etc. DO NOT DO IT. FTC robot code relies on a specific version and Java environment and changing it will cause the project to break.

## 3. Do not sync fork
Do NOT press the "sync fork" button on the home web page, merge from the upstream in the terminal, or pull any changes from the upstream repository. Since we are changing the files from the default template, doing so will revert our files back to the original, removing progress.

## 4. Do not update Gradle or anything else related to the SDK
If Android Studio or VSCode asks you to update Gradle, migrate to Gradle Daemon, etc. DO NOT DO IT. FTC robot code relies on a specific version and Java environment and changing it will cause the project to break.

## 5. Branching

To make changes, such as adding a feature or editing code, you must create a branch first. You are strictly blocked from pushing code directly to the `main` branch. 

* **Rule:** Always create a new branch from the latest `main` branch before you start making changes.
* **Naming Convention:** Use clear names for your branches so we know what you are doing:
  * For features: `feature/your-feature-name` (e.g., `feature/login-page`)
  * For bug fixes: `bugfix/your-fix-name` (e.g., `bugfix/broken-button`)
* <u>**Commits:** Please use precise and descriptive titles and explanations for each commit to clearly describe what you changed.</u>

## 6. Pull Request & Merging Rules

Once your work is finished on your side branch, you must open a Pull Request (PR) on GitHub.

* **Merge into <u>our</u> repository:** At the top of your pull request, make sure the leftmost dropdown in the top row shows "base: master" or "base repository: Lightspeed1024/FTC-Mechanical-Masters-Code", NOT "base repository: FIRST-Tech-Challenge/FtcRobotController". If it says the latter, click the dropdown and choose the one that says "Lightspeed1024/FTC-Mechanical-Masters-Code". Otherwise, you will be trying to edit the parent FTC repository and your PR will get blocked.
* **Merge Commits:** We strongly recommend using the default **Merge Commit** option on GitHub. Do not squash your commits and do not rebase, unless absolutely necessary. We want to preserve your full step-by-step history log.
* **Required Approvals:** You must receive approval from at least one member of the team before merging your pull request.

## 7. Do not update Gradle or anything else related to the SDK
If Android Studio or VSCode asks you to update Gradle, migrate to Gradle Daemon, etc. DO NOT DO IT. FTC robot code relies on a specific version and Java environment and changing it will cause the project to break.

## 8. If you accidentally committed to main:

**Step 1: Create Your New Feature Branch**

1. Look at the top of your GitHub Desktop screen and click on **Current Branch: main**.
2. Click the **New Branch** button.
3. Name your branch (e.g., `feature/my-changes`) and click **Create Branch**.

This safely copies your accidental commit over to your new feature branch.

**Step 2: Switch Back to Main to Clean It Up**

1. Click on **Current Branch** at the top of the screen again.
2. Select **main** from the list to switch back to it.

**Step 3: Undo the Mistake on Main**

1. In the top-left sidebar of GitHub Desktop, click on the **History** tab.
2. Look at the very top of the list for the accidental commit you just made.
3. Right-click that commit and select **Undo commit**.

This completely erases the accidental commit from your local `main` branch, making it perfectly clean again.

**Step 4: Push to Github**

1. Click on **Current Branch** one last time and switch back to your new feature branch (`feature/my-changes`).
2. Click the **Push origin** button at the top of the screen.
3. Go to GitHub.com and open your Pull Request!

---

Thank you for following these rules and maintaining a clean, organized repository!
