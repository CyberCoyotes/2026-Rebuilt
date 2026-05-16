## The rule to internalize:

- IntakeIO declares what hardware can do (the contract)
- IntakeIOHardware implements that contract
- IntakeSubsystem wraps IO methods as named Java methods, then command factories call those methods
- Command factories never call other command factories

io methods  →  subsystem low-level methods  →  command factories
