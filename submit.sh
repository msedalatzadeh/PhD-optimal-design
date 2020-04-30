#!/bin/bash
# bash script for submitting a Matlab job to the sharcnet Graham queue

#SBATCH --mem-per-cpu=1G        # memory per processor (default in Mb)
#SBATCH --time=00-02:00         # time (DD-HH:MM)
#SBATCH --job-name="Sajjad"     # job name
#SBATCH --input=ResultsGenerator6.m   # Matlab script
##SBATCH —dependency=afterok:<jobid>  # Wait for job to complete

#SBATCH --ntasks=1              # number of processors
#SBATCH --cpus-per-task=15
#SBATCH --output=log1.log                 # log file
#SBATCH --error=errorfile.err                  # error file
#SBATCH --mail-user=FAIL     # who to email
#SBATCH --mail-type=FAIL                    # when to email
#SBATCH --account=def-kmorris              # UW Fluids designated resource allocation
module load matlab/2018a 
srun matlab -nodisplay -nosplash