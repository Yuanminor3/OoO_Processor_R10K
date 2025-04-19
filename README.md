# CSEE W4824: Computer Architecture Final Project

## Clock period

1. Adjust it in Makefile

## Run Simulation

### 🔹 Run a Single Test Program

To test one specific program located in the `programs/` folder:

1. Open your terminal and navigate to your project directory:
   ```bash
   cd /path/to/your/project
   ```
2. Modify the `program=dft` in `sim_test.sh` script to specify the name of the program you want to test.
3. Run the script:
   ```bash
   sh sim_test.sh
   ```
   This will:
   - Automatically execute `make XXX.out`
   - Compare the generated memory output with the reference in the `sim_ground_truth/` folder.
4. The result will display **"pass"** or **"failed"** in the terminal.
5. To clean all generated files:
   ```bash
   make nuke
   ```

---

### 🔹 Run All 33 Test Programs

To test all programs in the `programs/` folder at once:

1. Navigate to your project directory:
   ```bash
   cd /path/to/your/project
   ```
2. Run the batch testing script:
   ```bash
   sh sim_test_all.sh
   ```
   This will:
   - Automatically run `make XXX.out` for each program
   - Compare each output to the ground truth files in the `sim_ground_truth/` folder.
3. You will see a summary of results, e.g., **33/33 passed**.
4. To avoid cluttering the terminal, detailed output and the final score will also be saved to:
   ```
   sim_all_results.txt
   ```
5. To clean all generated files, including the result log:
   ```bash
   make nuke
   ```
---

## Run Synthesis (takes a few hours)

### 🔹 Run a Single Test Program

To test one specific program located in the `programs/` folder:

1. Open your terminal and navigate to your project directory:
   ```bash
   cd /path/to/your/project
   ```
2. Modify the `program=dft` in `syn_test.sh` script to specify the name of the program you want to test.
3. Run the script:
   ```bash
   sh syn_test.sh
   ```
   This will:
   - Automatically execute `make XXX.syn.out`
   - Compare the generated memory output with the reference in the `syn_ground_truth/` folder.
4. The result will display **"pass"** or **"failed"** in the terminal.
5. To clean all generated files:
   ```bash
   make nuke
   ```

---

### 🔹 Run All 33 Test Programs

To test all programs in the `programs/` folder at once:

1. Navigate to your project directory:
   ```bash
   cd /path/to/your/project
   ```
2. Run the batch testing script:
   ```bash
   sh syn_test_all.sh
   ```
   This will:
   - Automatically run `make XXX.syn.out` for each program
   - Compare each output to the ground truth files in the `syn_ground_truth/` folder.
3. You will see a summary of results, e.g., **33/33 passed**.
4. To avoid cluttering the terminal, detailed output and the final score will also be saved to:
   ```
   syn_all_results.txt
   ```
5. To clean all generated files, including the result log:
   ```bash
   make nuke
   ```
---

## TBD