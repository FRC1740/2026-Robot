This is the official contribution style guide for the 2026 FRC 1740 robotics team. 

We use spaces with ideally 4 space spacing.

```java
here
    is
        4
            space
                indentation
```

Each scope change designates one more indentation block, ideally capping at 3
```java
class Example {
    void someFunction() {
        // someCodeIndentedTwice
    }
}
```

All variables should be written as ``camelCase``
```java
iAmAVariable = true;
```

Local copies of instances should be marked with ``m_``

```java
void IntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsyetm; // note camel case is applied AFTER m_
}
```

Constants should be prefixed with ``k``

```java
const int kDriverButton = 1;
```

All subsystems should implement a ``getInstance()`` function

```java
public static Climber getInstance() {
    if(instance == null) {
        instance = new Climber();
    }
    return instance;
}
```

Variables should *Almost Never* be accessed outisde their class, therefore most variables should be private. If a variable is const, mark it as final.
```java
private final someVariable = 0; // constant
private someVariableThatChanges = 0; // non-constant
```

Every function should be documented via javadoc, with the return, being as detailed as possible
```java
    /**
     * Function to retrive the current draw of the climber wheel.
     * 
     * @return wheel current draw in amps
     */
    public double getWheelCurrentDraw() {
        return climberWheel.getOutputCurrent();
    }
```