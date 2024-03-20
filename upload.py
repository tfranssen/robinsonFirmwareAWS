#Script to upload firmware to S3 bucket
import boto3
import os
import sys

firmwarePath = "./build/zephyr/app_update.bin"
firmware_name = input("Enter the name of the firmware file: ")
version = input("Enter the version of the firmware: ")
# check if there are dots and replace them with underscores
versionParsed = version.replace(".", "_")
firmware_name = firmware_name + versionParsed + os.path.splitext("./build/zephyr/app_update.bin")[1]
#copy file using os
os.system("cp " + firmwarePath + " " + firmware_name)
#Create a AWS IoT core Job  JS file with the same name as the firmware file
#first strip the extenstion
firmware_nameParsed = os.path.splitext(firmware_name)[0]
jobName = firmware_nameParsed + ".json"
#Create the file
jobFile = open(jobName, "w")
#Write the file
# {
#   "operation": "app_fw_update",
#   "fwversion": "v1.81",
#   "size": 181124,
#   "location": {
#     "protocol": "http:",
#     "host": "iotfotabucket.s3.eu-central-1.amazonaws.com",
#     "path": "app_update_1_8_1_lidarOnly.bin"
#   }
# }
jobFile.write("{\n")
jobFile.write("  \"operation\": \"app_fw_update\",\n")
jobFile.write("  \"fwversion\": \"" + version + "\",\n")
jobFile.write("  \"size\": " + str(os.path.getsize(firmware_name)) + ",\n")
jobFile.write("  \"location\": {\n")
jobFile.write("    \"protocol\": \"http:\",\n")
jobFile.write("    \"host\": \"iotfotabucket.s3.eu-central-1.amazonaws.com\",\n")
jobFile.write("    \"path\": \"" + firmware_name + "\"\n")
jobFile.write("  }\n")
jobFile.write("}\n")
jobFile.close()
#Upload the firmware to the S3 bucket
s3 = boto3.client('s3')
s3.upload_file(firmware_name, 'iotfotabucket', firmware_name)
#Upload the job file to the S3 bucket
s3.upload_file(jobName, 'iotfotabucket', jobName)
