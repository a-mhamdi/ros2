# Introduction to Shell Scripting

The **shell** provides a way to interact with the operating system, execute programs, combine commands, and automate repetitive tasks.

## 1. The Shell

A **shell** is a program that interprets commands and executes them.

Common shells include:

* Bash
* Zsh
* Fish

Check the current shell:

```bash
echo $SHELL
```

Check the Bash version:

```bash
bash --version
```

## 2. Navigation

### Print the current directory

Show the current working directory.

```bash
pwd
```

### List files

```bash
ls
```

More detailed information:

```bash
ls -l
```

Include hidden files:

```bash
ls -la
```

### Change directory

```bash
cd directory
```

Go to the parent directory:

```bash
cd ..
```

Go to the home directory:

```bash
cd ~
```

## 3. Files and Directories

### Create a file

```bash
touch file.txt
```

### Create a directory

```bash
mkdir project
```

Create nested directories:

```bash
mkdir -p directory/subdirectory
```

### Copy a file

```bash
cp file backup
```

Copy a directory:

```bash
cp -r project project_backup
```

### Move or rename a file

```bash
mv old.txt new.txt
```

Move a file:

```bash
mv file.txt project/
```

### Remove a file

```bash
rm file.txt
```

Remove a directory:

```bash
rm -r project
```

> [!WARNING]
> Be careful with `rm`: removed files are normally not moved to a recycle bin.

## 4. Viewing Files

### Display a file

```bash
cat file.txt
```

### View a large file

```bash
less logfile.txt
```

Useful keys inside `less`:

* `Space` — next page
* `b` — previous page
* `/text` — search
* `q` — quit

### Display the beginning of a file

```bash
head file.txt
```

first 20 lines:

```bash
head -n 20 file.txt
```

### Display the end of a file

```bash
tail file.txt
```

last 20 lines:

```bash
tail -n 20 file.txt
```

Follow a file as it changes:

```bash
tail -f file.txt
```

> This is particularly useful for **logs**.

## 5. Getting Help

### Manual pages

```bash
man ls
```

Manual pages provide detailed documentation for commands.

### Command help

```bash
ls --help
```

Many commands provide a shorter help message using `--help`.

### Find the location of a command

```bash
which python
```

or:

```bash
which bash
```

## 6. Searching

### Search for files

```bash
find . -name "*.txt"
```

Search from the current directory for files ending in `.txt`.

### Search for text

```bash
grep "error" logfile.txt
```

Search recursively:

```bash
grep -r "error" .
```

### Command history

```bash
history
```

Execute a previous command using its history number:

```bash
!123
```

## 7. Redirection

**Shell** commands normally produce output on **standard output**.

### Redirect output to a file

```bash
ls > files.txt
```

The output is written to `files.txt`.

### Append output

```bash
ls >> files.txt
```

The output is added to the end of the file.

### Redirect input

```bash
sort < names.txt
```

The contents of `names.txt` become the input of `sort`.

### Redirect errors

```bash
command 2> error.log
```

Standard error is redirected to `error.log`.

## 8. Pipes

A pipe `|` sends the output of one command to another command.

```bash
ls | grep ".txt"
```

This means:

**`ls` → output → `grep` → filtered output**

Pipes allow simple commands to be combined. For example:

```bash
cat logfile.txt | grep "ERROR"
```

Or:

```bash
ps | grep python
```

## 9. Variables

Create a variable:

```bash
name="Ali"
```

Use the variable:

```bash
echo "$name"
```

Display an environment variable:

```bash
echo "$HOME"
```

Another useful variable:

```bash
echo "$PATH"
```

The `PATH` variable contains directories where the shell searches for executable commands.

## 10. Command Substitution

The output of a command can be stored in a variable.

```bash
current_dir=$(pwd)
```

Then:

```bash
echo "$current_dir"
```

This is very useful when building scripts from several commands.

## 11. Exit Status

Commands return an **exit status**.

```bash
ls
echo $?
```

Convention:

* `0` → success
* Non-zero → failure

For example:

```bash
ls existing_file.txt
echo $?
```

The exit status allows a script to determine whether a command succeeded.

## 12. Basic Conditions

A shell script can make decisions using `if`. Conceptually:

```text
if condition
    do something
else
    do something else
```

A simple file test:

```bash
if [ -f file.txt ]; then
    echo "File exists"
fi
```

Check whether a directory exists:

```bash
if [ -d project ]; then
    echo "Directory exists"
fi
```

## 13. Loops

### `for`

Repeat an operation for several values:

```bash
for file in *.txt; do
    echo "$file"
done
```

### `while`

Repeat while a condition is true:

```bash
while condition; do
    # commands
done
```

## 14. Script Arguments

A script can receive arguments from the command line. For example:

```bash
./script.sh file.txt
```

Inside the script:

```bash
$0
```

represents the script name.

```bash
$1
```

represents the first argument.

```bash
$2
```

represents the second argument.

```bash
$#
```

contains the number of arguments.

```bash
$@
```

represents all arguments.

## 15. User Input

A script can ask the user for input.

```bash
read name
```

The value entered by the user is stored in `name`. A common pattern is:

```text
display a message
read the user's input
process the input
```

## 16. Functions

Functions allow us to group commands and reuse them.

Basic structure:

```bash
function_name() {
    # commands
}
```

Call the function:

```bash
function_name
```

Functions can also receive arguments:

```bash
function_name argument1 argument2
```

## 17. Permissions

Display file permissions:

```bash
ls -l
```

Make a script executable:

```bash
chmod +x script.sh
```

Then execute it:

```bash
./script.sh
```

## 18. Processes

Display running processes:

```bash
ps
```

Display processes interactively:

```bash
top
```

Search for a process:

```bash
ps | grep python
```

Terminate a process:

```bash
kill PID
```

where `PID` is the process ID.

## 19. Background Processes

Run a command in the background:

```bash
command &
```

Show shell jobs:

```bash
jobs
```

Bring a background job to the foreground:

```bash
fg
```

## 20. Text Processing

### `grep`

Search for text:

```bash
grep "hello" file.txt
```

### `wc`

Count lines:

```bash
wc -l file.txt
```

### `sort`

Sort lines:

```bash
sort names.txt
```

### `uniq`

Remove consecutive duplicate lines:

```bash
sort names.txt | uniq
```

### `cut`

Extract fields:

```bash
cut -d ',' -f 1 data.csv
```

### Combining commands

The real power comes from combining tools:

```bash
cat data.txt | grep "ERROR" | sort | uniq
```

Each command performs a small operation, and the pipeline combines them into a larger operation.

## 21. Scheduling

`cron` allows commands and scripts to run automatically at scheduled times.

View scheduled jobs:

```bash
crontab -l
```

Edit scheduled jobs:

```bash
crontab -e
```

Typical applications include:

* Backups
* Periodic cleanup
* Monitoring
* Data processing
* Maintenance

## 22. Script Workflow

A typical shell scripting workflow is:

```text
1. Identify a repetitive task
        ↓
2. Find commands that perform the task
        ↓
3. Combine commands using pipes/redirection
        ↓
4. Add variables and conditions
        ↓
5. Put the commands into a script
        ↓
6. Make the script executable
        ↓
7. Automate its execution
```

> [!IMPORTANT]
> **Shell** scripting isn't primarily about programming; it's about orchestrating existing programs.

## 23. TMUX - Windows & Panes

`tmux` allows us to keep multiple terminal sessions running inside a single terminal.

> [!NOTE]
> `Ctrl+b` is the `tmux` prefix. Most `tmux` shortcuts start with it.

```text
Session
 ├── Window
 │    ├── Pane
 │    └── Pane
 └── Window
      └── Pane
```
### Windows

Create a new window:

```text
Ctrl+b  c
```

Switch to the next window:

```
Ctrl+b  n
```

Switch to the previous window:

```
Ctrl+b  p
```

Switch directly to a window:

```text
Ctrl+b  0
Ctrl+b  1
Ctrl+b  2
```

### Panes

Split vertically _(left/right)_:

```text
Ctrl+b  %
```

Split horizontally _(top/bottom)_:

```text
Ctrl+b  "
```

Move between panes:

```text
Ctrl+b  ←
Ctrl+b  →
Ctrl+b  ↑
Ctrl+b  ↓
```
