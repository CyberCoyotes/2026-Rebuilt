# 2025-2026-Rebuilt

**Cyber Coyotes** codebase for the 2025-2026 FRC season *Rebuilt*

## Contributing

This repository uses branch protection on the `main` branch to ensure code quality and prevent unwanted commits. All changes must be made through pull requests.

### Getting Started

1. Fork or clone this repository
2. Create a feature branch: `git checkout -b feature/your-feature-name`
3. Make your changes and commit them
4. Push to your branch: `git push origin feature/your-feature-name`
5. Open a Pull Request against the `main` branch

### Branch Protection

The `main` branch is protected with the following requirements:
- All changes must be made via pull requests
- Pull requests require at least one approval
- All CI checks must pass before merging
- Code reviews from designated owners are required

For more details on branch protection and how to configure it, see [.github/BRANCH_PROTECTION.md](.github/BRANCH_PROTECTION.md).

### CI/CD

This repository uses GitHub Actions for continuous integration:
- **CI Workflow**: Builds and tests the code on every PR and push to main
- **PR Validation**: Validates pull request quality and checks for conflicts

See the [.github/workflows](.github/workflows) directory for workflow configurations.

## Building and Testing

```bash
# Build the project
./gradlew build

# Run tests
./gradlew test

# Deploy to robot
./gradlew deploy
```
