# Testing and Quality Assurance

This assignment is concerned with software testing and (code) quality assurance. The assignment builds on your first course assignment (where you worked with state machines and behaviour trees), such that the objective is to embed the software you developed there into a continuous integration workflow where your software is tested and the quality of the code is evaluated.

The following are your concrete tasks in this assignment:

1. **Set up a GitHub repository** and push your code from the assignment on state machines and behaviour trees there (or use an existing repository if you already have your code there).
2. **Implement some tests for your components** (unit, but also integration). Feel free to decide which aspects of your code you will test (e.g. whether you handle the sensor data correctly, whether your state machine / behaviour tree has the correct states / behaviours, etc.). For testing, I recommend using the [unittest](https://docs.python.org/3/library/unittest.html) Python library, but you can also explore other alternatives if you want.
3. **Set up continuous integration in your repository**. The continuous integration process should execute your unit tests so that you can ensure that any changes made to the implementation do not break the tests. Since we are using GitHub, you should use [GitHub CI](https://docs.github.com/en/actions/automating-builds-and-tests/about-continuous-integration) for this purpose.
4. Software testing can ensure that your code is functionally correct, but cannot ensure that it is well-written or that it follows concrete development standards. Thus, in addition to continuous integration, we need some way of verifying the code quality in your software. For this purpose, you need to **set up** [**Codacy**](https://docs.codacy.com/) **in your repository**. At the very least, you should set it up so that it verifies that your code follows the [PEP8 style guide](https://peps.python.org/pep-0008/) for Python, and should warn you about complex and repeated code. Set up Codacy so that it is executed for every pull request you create to your main branch.
5. In your final task, you need to:
    1. **Develop some new functionality for your robot**; feel free to be creative about this. The functionality should be developed on a feature branch in your repository.
    2. **Update your unit tests so that the new components are covered by the tests**.
    3. Once you are done with the implementation, **create a pull request and merge it only once you have resolved the issues raised by Codacy**.

To test the robustness of your workflow, you can, for instance, push some changes that should break some of your tests and create a pull request with low-quality code that should be flagged by Codacy.