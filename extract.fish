#!/bin/fish

# rsync -avz --mkpath hulk@bighulk.hulks.dev:/mnt/raid/disk/logs/(git branch --show-current) ~/worktree/logs

for folder in ~/worktree/logs/(git branch --show-current)/*/*/*/
   ./pepsi run imagine -- $folder (string replace logs images $folder)
end

