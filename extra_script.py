Import("env")
folder = env.GetProjectOption("src_folder")

# Generic
env.Replace(
    PROJECT_SRC_DIR="$PROJECT_DIR/src/" + folder
)
