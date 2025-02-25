# This script removes or comments out all Python calls to aid troubleshooting

# Inputs ------------------------------------------
job_dir = "./Mount/"
job_file = "M_PRINT.JBI"

mode = "D" #delete
# mode = "C" #comment

# Outputs ------------------------------------------
# Files starting with the mode prefix



# Code ---------------------------------------------
job_dir_loc = job_file.split('/')
currentLineNumber = 0
fileListStart = None
outfile = ""
fileList = []
# Open main job file that calls the other jobs
# Create list of files for other jobs
# Change name to reflect mode
with open(job_dir + job_file,'r') as job:
    for line in job:
        ln = line.strip()
        if ln == "NOP":
            fileListStart = currentLineNumber
        elif fileListStart == None:
            if "//NAME" in ln:
                ln = "//NAME "+ mode + ln[7:len(ln)]
            ## in the header blocks
        elif ln != "END":
            ## List of files to call
            ln = ln.replace("PRINT3D",mode+"PRINT3D")
            fileList.append(ln.replace("CALL JOB:","")+".JBI")
        outfile += ln + "\n"
        
# Write new job file
with open(job_dir + "outputs/" + mode + job_file,'w') as job:
    job.write(outfile)

# Go through other job files
# Remove calls
# Rename to reflect mode
for fname in fileList:
    out_job_file = ""
    with open(job_dir + fname[1:len(fname)],"r") as job:
        for line in job:
            ln = line.strip()
            if mode == "replace":
                ln = ln.replace("CALL","'CALL")
            elif mode == "D":
                if "CALL JOB" in ln:
                    continue
            if "//NAME" in ln:
                ln = "//NAME "+ mode + ln[7:len(ln)]
            out_job_file += ln + "\n"
    # Write the new job file
    with open(job_dir + "outputs/" +fname,"w") as job:
        job.write(out_job_file)