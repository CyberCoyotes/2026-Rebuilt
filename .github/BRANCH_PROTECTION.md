# Branch Protection Setup Guide

This document explains how to configure branch protection for the `main` branch to prevent unwanted commits.

## GitHub Workflows

The following GitHub Actions workflows have been set up to support branch protection:

1. **CI Workflow** (`.github/workflows/ci.yml`)
   - Runs on every push to `main` and on all pull requests
   - Builds the robot code using Gradle
   - Runs all tests
   - Checks code formatting
   - Uploads build artifacts and test results

2. **PR Validation Workflow** (`.github/workflows/pr-validation.yml`)
   - Validates pull request titles
   - Checks for merge conflicts
   - Ensures PRs are not created from the main branch

## Required GitHub Branch Protection Settings

To fully protect the `main` branch, configure the following settings in GitHub:

### Steps to Configure:

1. Go to your repository on GitHub
2. Click **Settings** → **Branches**
3. Under "Branch protection rules", click **Add rule**
4. Configure the following settings:

#### Branch Name Pattern
```
main
```

#### Protection Settings

**Protect matching branches:**
- ✅ **Require a pull request before merging**
  - ✅ Require approvals: **1** (or more, depending on team size)
  - ✅ Dismiss stale pull request approvals when new commits are pushed
  - ✅ Require review from Code Owners

- ✅ **Require status checks to pass before merging**
  - ✅ Require branches to be up to date before merging
  - **Required status checks:**
    - `build` (from CI workflow)
    - `validate-pr` (from PR Validation workflow)

- ✅ **Require conversation resolution before merging**

- ✅ **Require signed commits** (optional, but recommended)

- ✅ **Require linear history** (optional, prevents merge commits)

- ✅ **Do not allow bypassing the above settings**

- ✅ **Restrict who can push to matching branches**
  - Add only: Repository administrators and specific CI/CD service accounts

**Rules applied to everyone including administrators:**
- ✅ **Require status checks to pass**
- ✅ **Require pull request reviews**

## CODEOWNERS

A `.github/CODEOWNERS` file has been created to automatically request reviews from appropriate team members. Update the teams/users in this file to match your organization structure:

- `@CyberCoyotes/maintainers` - Default reviewers for all code
- `@CyberCoyotes/robot-leads` - Required for robot code changes
- `@CyberCoyotes/mentors` - Required for build configuration changes
- `@CyberCoyotes/admins` - Required for GitHub workflow changes

## Benefits of Branch Protection

With these settings enabled:

1. **No direct commits to main** - All changes must go through pull requests
2. **Code review required** - At least one approval needed before merging
3. **Automated testing** - CI workflows must pass before merging
4. **Conflict prevention** - Branches must be up-to-date with main
5. **Quality assurance** - Automated validation of PR quality

## Testing the Setup

To test that branch protection is working:

1. Try to push directly to `main` (should be blocked)
2. Create a feature branch: `git checkout -b test-feature`
3. Make a small change and push: `git push -u origin test-feature`
4. Create a pull request on GitHub
5. Verify that CI workflows run automatically
6. Verify that merge is blocked until checks pass and reviews are approved

## Maintaining the Workflows

- CI workflow runs on every PR and push to main
- Update the workflows as needed when adding new tools or checks
- Monitor workflow runs in the **Actions** tab on GitHub
- Fix any failing checks promptly to maintain code quality

## Additional Recommendations

1. **Enable branch protection early** - Set it up before accepting contributions
2. **Educate the team** - Ensure everyone understands the PR workflow
3. **Document coding standards** - Create contributing guidelines
4. **Review regularly** - Periodically review and update protection rules
5. **Use draft PRs** - For work-in-progress changes that aren't ready for review

## Support

If you have questions about branch protection or the CI workflows, please:
- Check the GitHub Actions logs for workflow failures
- Review this documentation
- Contact the repository maintainers

---

**Note:** Branch protection rules must be configured through the GitHub web interface. The workflows and CODEOWNERS file in this repository provide the foundation, but the actual protection rules are repository settings.
