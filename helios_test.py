import zivid

#Save file
app = zivid.Application()
camera = app.create_file_camera("./script/FileCameraZividOne.zfc")
settings = zivid.Settings(acquisitions=[zivid.Settings.Acquisition()])
frame = camera.capture(settings)
#frame.save("result.zdf")

data_file_ply = "PointCloud.ply"
frame.save(data_file_ply)



#Read file
# data_file = get_sample_data_path() / "result.zdf"
# print(f"Reading point cloud from file: {data_file}")
# frame = zivid.Frame(data_file)