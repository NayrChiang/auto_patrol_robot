# Contributing to Autonomous Patrol Robot

Thank you for your interest in contributing to the Autonomous Patrol Robot project! This document provides guidelines and instructions for contributing.

## Code of Conduct

By participating in this project, you agree to maintain a respectful and inclusive environment for all contributors.

## How to Contribute

### Reporting Bugs

If you find a bug, please open an issue with:
- A clear, descriptive title
- Steps to reproduce the issue
- Expected behavior vs. actual behavior
- System information (OS, ROS 2 version, etc.)
- Relevant log files or error messages

### Suggesting Enhancements

We welcome suggestions for new features or improvements. Please open an issue with:
- A clear description of the proposed enhancement
- Use cases and benefits
- Possible implementation approach (if you have ideas)

### Pull Requests

1. **Fork the repository** and create a new branch from `main`
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** following our coding standards:
   - Follow ROS 2 coding conventions
   - Add comments for complex logic
   - Update documentation as needed
   - Ensure code passes linting checks

3. **Test your changes**:
   - Build the workspace: `colcon build --symlink-install`
   - Test in simulation before submitting
   - Verify no regressions in existing functionality

4. **Commit your changes**:
   ```bash
   git commit -m "Add: brief description of changes"
   ```
   Use clear, descriptive commit messages.

5. **Push to your fork**:
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Open a Pull Request**:
   - Provide a clear description of changes
   - Reference any related issues
   - Include screenshots or demos if applicable

## Coding Standards

### Python
- Follow PEP 8 style guide
- Use meaningful variable and function names
- Add docstrings for functions and classes
- Maximum line length: 100 characters

### ROS 2
- Follow ROS 2 naming conventions
- Use appropriate message types
- Handle errors gracefully with proper logging
- Document launch file parameters

### Git Commit Messages
- Use present tense ("Add feature" not "Added feature")
- Keep the first line under 50 characters
- Add detailed description if needed
- Reference issues: "Fix #123"

## Development Setup

1. Clone your fork:
   ```bash
   git clone https://github.com/your-username/auto_patrol_robot.git
   cd auto_patrol_robot
   ```

2. Set up ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace:
   ```bash
   cd autopatrol_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Run tests (if available):
   ```bash
   colcon test
   ```

## Project Structure

- `autopatrol_ws/src/robot_description/` - Robot model and URDF files
- `autopatrol_ws/src/robot_navigation2/` - Navigation2 configuration
- `autopatrol_ws/src/autopatrol_robot/` - Main patrol application
- `autopatrol_ws/src/autopatrol_interfaces/` - Custom service interfaces

## Areas for Contribution

- Bug fixes and error handling improvements
- Documentation enhancements
- Performance optimizations
- Additional sensor integrations
- New navigation features
- Test coverage improvements
- Code refactoring and cleanup

## Questions?

If you have questions, please:
- Open an issue with the "question" label
- Check existing issues and discussions
- Review the README.md for project overview

Thank you for contributing! 🚀

