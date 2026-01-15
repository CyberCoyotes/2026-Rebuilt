# Contributing to 2026-Rebuilt

Thank you for your interest in contributing to the Cyber Coyotes 2026 robot code! This document provides guidelines for contributing to this repository.

## Getting Started

1. **Fork the repository** (if you're not a team member)
2. **Clone your fork** or the main repository
   ```bash
   git clone https://github.com/CyberCoyotes/2026-Rebuilt.git
   cd 2026-Rebuilt
   ```
3. **Set up your development environment**
   - Install [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
   - Install Java 17 (included with WPILib)

## Making Changes

### Branch Protection

The `main` branch is protected. All changes must go through pull requests. Direct commits to `main` are not allowed.

### Creating a Feature Branch

Always create a new branch for your changes:

```bash
git checkout -b feature/your-feature-name
```

Branch naming conventions:
- `feature/` - New features
- `fix/` - Bug fixes
- `docs/` - Documentation updates
- `refactor/` - Code refactoring
- `test/` - Adding or updating tests

### Making Commits

- Write clear, descriptive commit messages
- Keep commits focused on a single change
- Reference issue numbers when applicable

```bash
git add .
git commit -m "Add intake subsystem with motor control"
```

### Testing Your Changes

Before submitting a pull request, ensure:

1. **Code builds successfully**
   ```bash
   ./gradlew build
   ```

2. **All tests pass**
   ```bash
   ./gradlew test
   ```

3. **Code follows team standards**
   - Use proper naming conventions
   - Add comments for complex logic
   - Follow existing code style

4. **Test on robot or simulator** (when applicable)
   ```bash
   ./gradlew simulateJava
   ```

## Submitting a Pull Request

1. **Push your branch** to GitHub
   ```bash
   git push origin feature/your-feature-name
   ```

2. **Create a Pull Request**
   - Go to the repository on GitHub
   - Click "New Pull Request"
   - Select your branch
   - Fill out the PR template completely

3. **PR Requirements**
   - Descriptive title and description
   - Link to related issues
   - All CI checks must pass
   - At least one approval from a code owner
   - No merge conflicts with `main`

4. **Address Review Feedback**
   - Respond to comments
   - Make requested changes
   - Push updates to your branch

## Code Review Process

1. **Automatic Checks**
   - CI workflow builds the code
   - Tests are run automatically
   - PR validation checks for conflicts

2. **Human Review**
   - Code owners will review your changes
   - Reviews typically happen within 1-2 days
   - Be responsive to feedback

3. **Merging**
   - Once approved and all checks pass, a maintainer will merge your PR
   - Your branch will be deleted after merging

## Code Style Guidelines

- **Java Code**
  - Use camelCase for variables and methods
  - Use PascalCase for class names
  - Add JavaDoc comments for public methods
  - Keep methods small and focused

- **File Organization**
  - Subsystems go in `src/main/java/frc/robot/subsystems/`
  - Commands go in `src/main/java/frc/robot/commands/`
  - Constants go in appropriate constant files

- **Comments**
  - Explain WHY, not WHAT
  - Use comments for complex algorithms
  - Keep comments up-to-date with code changes

## Getting Help

- **Questions about code?** Ask in team meetings or Slack
- **Found a bug?** [Open an issue](../../issues/new)
- **Need help with WPILib?** Check the [WPILib documentation](https://docs.wpilib.org/)
- **Git problems?** Ask a mentor or team lead

## Important Notes

- Never commit sensitive information (passwords, API keys, etc.)
- Test your changes before submitting a PR
- Keep PRs focused - one feature or fix per PR
- Be respectful in code reviews and discussions

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [FRC Game Manual](https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system)
- [Branch Protection Guide](.github/BRANCH_PROTECTION.md)
- [GitHub Flow Guide](https://guides.github.com/introduction/flow/)

Thank you for contributing to our robot code! 🤖
