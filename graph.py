# NOTE - this script was entirely created with Github Copilot Agent mode 
# using Gemini 2.5 Pro.

import matplotlib.pyplot as plt
import numpy as np
import re

def parse_results(filepath):
    """Parses the results.txt file."""
    boids = []
    cpu_times = []
    cuda_times = []
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            i = 0
            while i < len(lines):
                if lines[i].strip().startswith('N boids'):
                    try:
                        n_boids_match = re.search(r'=\s*(\d+)', lines[i])
                        if not n_boids_match:
                            print(f"Skipping block near line {i+1}: Could not parse N boids.")
                            i += 1
                            continue

                        n_boids = int(n_boids_match.group(1))

                        # Ensure next lines exist before accessing
                        if i + 2 < len(lines):
                            cpu_line = lines[i+1]
                            cuda_line = lines[i+2]
                            # Check if lines contain expected patterns
                            cpu_match = re.search(r'CPU time\s*=\s*([\d.]+)', cpu_line)
                            cuda_match = re.search(r'CUDA time\s*=\s*([\d.]+)', cuda_line)

                            if cpu_match and cuda_match:
                                cpu_time = float(cpu_match.group(1))
                                cuda_time = float(cuda_match.group(1))
                                boids.append(n_boids)
                                cpu_times.append(cpu_time)
                                cuda_times.append(cuda_time)
                                i += 4 # Move past the current block (N boids, CPU, CUDA, ----- or similar)
                            else:
                                print(f"Skipping block near line {i+1}: Could not find CPU or CUDA time pattern.")
                                i += 1 # Move to the next line
                        else:
                             print(f"Skipping incomplete block at end of file near line {i+1}.")
                             i = len(lines) # End loop
                    except (AttributeError, IndexError, ValueError) as e:
                        print(f"Skipping malformed block starting near line {i+1}: {e}")
                        i += 1 # Move to the next line to avoid infinite loop
                else:
                    i += 1
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
    except Exception as e:
        print(f"An unexpected error occurred while parsing {filepath}: {e}")
    return boids, cpu_times, cuda_times

def parse_results_usec_cuda(filepath):
    """Parses the results_usec_cuda.txt file."""
    boids = []
    cuda_times_usec = []
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            i = 0
            while i < len(lines):
                 if lines[i].strip().startswith('N boids'):
                    try:
                        n_boids_match = re.search(r'=\s*(\d+)', lines[i])
                        if not n_boids_match:
                            print(f"Skipping block near line {i+1}: Could not parse N boids.")
                            i += 1
                            continue
                        n_boids = int(n_boids_match.group(1))

                         # Ensure next line exists
                        if i + 1 < len(lines):
                            cuda_line = lines[i+1]
                            # Check if line contains expected pattern
                            cuda_match = re.search(r'CUDA time\s*=\s*([\d.]+)', cuda_line)

                            if cuda_match:
                                cuda_time = float(cuda_match.group(1))
                                boids.append(n_boids)
                                cuda_times_usec.append(cuda_time)
                                # Assuming 3 lines per block: N boids, CUDA time, ----
                                i += 3
                            else:
                                print(f"Skipping block near line {i+1}: Could not find CUDA time pattern.")
                                i += 1 # Move to the next line
                        else:
                            print(f"Skipping incomplete block at end of file near line {i+1}.")
                            i = len(lines) # End loop
                    except (AttributeError, IndexError, ValueError) as e:
                        print(f"Skipping malformed block starting near line {i+1}: {e}")
                        i += 1 # Move to the next line to avoid infinite loop
                 else:
                    i += 1
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
    except Exception as e:
        print(f"An unexpected error occurred while parsing {filepath}: {e}")
    return boids, cuda_times_usec

# --- Graph 1: results.txt (CPU in msec) ---
def plot_results(boids, cpu_times, cuda_times): # Keep cuda_times param for now, though unused
    if not boids:
        print("No data parsed from results.txt to plot.")
        return

    x = np.arange(len(boids))
    width = 0.6 # Adjusted width as there's only one bar set now

    fig, ax = plt.subplots(figsize=(12, 8)) # Increased height slightly for larger labels
    rects1 = ax.bar(x, cpu_times, width, label='CPU') # Centered bars
    # rects2 = ax.bar(x + width/2, cuda_times, width, label='CUDA') # Removed CUDA bars

    ax.set_ylabel('Average Time (msec)', fontsize=20) # Increased fontsize
    ax.set_xlabel('Number of Boids', fontsize=20) # Increased fontsize
    ax.set_title('CPU Performance (Time in msec)', fontsize=24) # Increased fontsize
    ax.set_xticks(x)
    ax.set_xticklabels(boids, rotation=45, ha="right", fontsize=16) # Increased fontsize
    ax.tick_params(axis='y', labelsize=16) # Increased y-tick label size
    # ax.legend() # Removed legend as only one data series
    ax.grid(axis='y', linestyle='--', alpha=0.7)

    # Add value labels
    ax.bar_label(rects1, padding=3, fmt='%.1f', fontsize=16) # Increased fontsize
    # ax.bar_label(rects2, padding=3, fmt='%.1f') # Removed CUDA labels

    fig.tight_layout()
    try:
        plt.savefig("results_cpu_msec_comparison.png") # Changed filename
        print("Generated results_cpu_msec_comparison.png") # Changed print message
    except Exception as e:
        print(f"Error saving results_cpu_msec_comparison.png: {e}")
    # plt.show()
    plt.close(fig)

# --- Graph 2: results_usec_cuda.txt (CUDA in msec) ---
def plot_results_usec_cuda(boids, cuda_times_usec):
    if not boids:
        print("No data parsed from results_usec_cuda.txt to plot.")
        return

    # Convert usec to msec
    cuda_times_msec = [t / 1000.0 for t in cuda_times_usec]
    
	# Convert the x-axis values to thousands for better readability
    boids = [n // 1000 for n in boids] # Convert to thousands

    x = np.arange(len(boids))
    width = 0.6

    fig, ax = plt.subplots(figsize=(12, 8)) # Increased height slightly for larger labels
    rects = ax.bar(x, cuda_times_msec, width, label='CUDA', color='orange') # Use converted times

    ax.set_ylabel('Average Time (msec)', fontsize=20) # Increased fontsize
    ax.set_xlabel('Number of Boids (Thousands)', fontsize=20) # Increased fontsize
    ax.set_title('CUDA Performance (Time in msec)', fontsize=24) # Increased fontsize
    ax.set_xticks(x)
    ax.set_xticklabels(boids, rotation=45, ha="right", fontsize=16) # Increased fontsize
    ax.tick_params(axis='y', labelsize=16) # Increased y-tick label size
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    # ax.legend()

    ax.bar_label(rects, padding=3, fmt='%.1f', fontsize=16) # Increased fontsize

    fig.tight_layout()
    try:
        plt.savefig("results_cuda_msec_comparison.png") # Changed filename
        print("Generated results_cuda_msec_comparison.png") # Changed print message
    except Exception as e:
        print(f"Error saving results_cuda_msec_comparison.png: {e}")
    # plt.show()
    plt.close(fig)


if __name__ == "__main__":
    # Parse data
    print("Parsing results.txt...")
    boids1, cpu_times, cuda_times_msec = parse_results('results.txt')
    print(f"Parsed {len(boids1)} entries from results.txt")

    print("\nParsing results_usec_cuda.txt...")
    boids2, cuda_times_usec = parse_results_usec_cuda('results_usec_cuda.txt')
    print(f"Parsed {len(boids2)} entries from results_usec_cuda.txt")

    # Generate plots
    print("\nGenerating plots...")
    plot_results(boids1, cpu_times, cuda_times_msec)
    plot_results_usec_cuda(boids2, cuda_times_usec)
    print("\nFinished.")
