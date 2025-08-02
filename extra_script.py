import subprocess

Import("env")
folder = env.GetProjectOption("src_folder")

# Generic
env.Replace(
    PROJECT_SRC_DIR="$PROJECT_DIR/src/" + folder
)



def get_git_revision_short_hash():
    try:
        return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('utf-8').strip()
    except Exception:
        return "unknown"

git_revision = get_git_revision_short_hash()

env.Append(CPPDEFINES=[
    ("GIT_REV", f'\\"{git_revision}\\"')
])

print(f"Current git revision: {git_revision}")
