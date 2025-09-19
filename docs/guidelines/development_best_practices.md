# Development Best Practices

### Generative AI Conventions

- If you're asking AI how to do something in ROS make sure to clarify that you're using ROS2 and not ROS1. If you see ChatGPT reccomending commands like `rosrun` and `roslaunch`, it means it's giving you ROS1 suggestions that simply won't work.

- If your looking for the AI to convert python to c++ or another language, dont ask for more than 3 lines at once. Errors get harder to spot the more code you ask for in the one prompt.

- At the end of the day, you want other technically minded people (including you) to understand what the code you make does. So if you end up copying and pasting code from ChatGPT, don't go expecting anyone else to understand it if you don't understand it yourself (even if it works!).

- AI can hallucinate pretty easily, so make sure your questions are clear and concise.

- An example of question ChatGPT misunderstood (it responded with code trying to make the function get_paramater() instead of answering the question):

    - `"how would I convert this code to c if the return value is a boolean self.get_parameter("use_slam").value"`

- Equivalent example it did understand *(with context from previous prompt):

    - `"what is the equivalent of self.get_parameter("use_slam").value in c++?"`

- The only problem with the understood prompt, is that its almost too simple.

- You have to remember, the AI in GPT isn't true intelligence, its more synonymous to a very complex string matching algorithm. Point is, **don't ever** expect the AI's reasoning to be correct. **At most** it as an educated guess.

- Also, for the times you are using ChatGPT specifically, you can use the **"Customize ChatGPT"** tab to configure how it reacts. i.e. you can tell it `"I'm a software developer from XXXX who wants concise answers and generally programs in XXXX but occasionally XXXX etc."`

### GitHub Conventions 

- Github repositories should be named `"ft_<NAME>"`. ROS2 Packages defined withing GitHub repositories should follow the same naming convention.
- Remember to push changes to a branch with a descriptive name and make pull requests when merging branches.
- Keep people up to date on what you're working on in slack channel.