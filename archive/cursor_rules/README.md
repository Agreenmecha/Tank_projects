# Cursor Project Rules

These rules provide persistent context to Cursor AI about the Tank project structure, workflows, and patterns.

## Available Rules

### 1. rover-access
**When to use:** When working with rover SSH operations, remote commands, or session management.

Contains:
- Rover connection details (IP, credentials)
- `./rover` command patterns
- Session management
- Common troubleshooting

### 2. ros2-workflow  
**When to use:** When working in `tank_ws/` or with ROS 2 packages.

Contains:
- Build procedures
- Launch patterns
- ROS 2 commands
- Package structure

### 3. tank-project-structure
**When to use:** When navigating the project or locating files.

Contains:
- Directory layout
- Hardware details
- Key file locations
- Network configuration

### 4. development-workflow
**When to use:** When developing, testing, or debugging.

Contains:
- Standard dev cycle
- Testing patterns
- Debugging workflows
- Git best practices

## How to Use

### Manual Application
Reference a rule in chat with @-mention:
```
@rover-access how do I start sensors?
```

### Intelligent Application
Rules with descriptions will be automatically applied by Cursor when relevant to your context.

### File-Specific Application
The `ros2-workflow` rule automatically applies when working in `tank_ws/`.

## Managing Rules

View and manage rules in:
**Cursor Settings → Rules, Commands**

Or edit directly:
`.cursor/rules/<rule-name>/RULE.md`

## Best Practices

- Use @-mentions when you need specific rule context
- Rules are version-controlled - commit changes
- Keep rules updated as workflows evolve
- Reference actual files with @filename syntax

## Rule Types

| Rule | Type | Trigger |
|------|------|---------|
| rover-access | Apply Intelligently | Description-based |
| ros2-workflow | Apply to Specific Files | `tank_ws/**/*` |
| tank-project-structure | Apply Intelligently | Description-based |
| development-workflow | Apply Intelligently | Description-based |

## Benefits

- ✅ Reduced context usage in conversations
- ✅ Consistent command patterns
- ✅ Quick reference for common operations
- ✅ Version-controlled knowledge base
- ✅ Shared understanding across sessions

## Updating Rules

When workflows change:

1. Edit `.cursor/rules/<rule-name>/RULE.md`
2. Commit changes: `git add .cursor/rules && git commit -m "Update rules"`
3. Push to GitHub
4. Pull on rover if needed

## Documentation References

Rules reference these docs:
- `scripts/ROVER_SSH_README.md`
- `scripts/ROVER_QUICK_REFERENCE.md`
- `DIRECTORY_STRUCTURE.md`
- `README.md`

---

**Created:** December 18, 2025  
**Purpose:** Efficient context management for Tank rover development

