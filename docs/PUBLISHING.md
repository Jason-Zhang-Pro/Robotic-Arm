# Publishing checklist

The repository is published at https://github.com/Jason-Zhang-Pro/Robotic-Arm. These notes cover source ownership and future release preparation.

- Choose the A4 version demonstrated successfully on your hardware, then document its calibration and cache generation procedure.
- Inspect MATLAB live-script metadata/output and retained figures/data for personal or team information. Text-only screening cannot establish that binary files are clear.
- Confirm permission to share team coursework and reused university helper code. Course libraries, handouts, reports and third-party CAD were excluded; remaining files still need provenance review before public release.
- Check local Python/FFmpeg paths, COM ports and missing SO101 mesh assets.
- Run the chosen simulation in MATLAB and validate hardware separately. Repository preparation only performed static checks.
- Choose GitHub repository name and visibility, then set its URL as the remote. Prefer private visibility while ownership and coursework sharing rules are unresolved.

From this repository directory, after reviewing the snapshot:

```powershell
git add .
git commit -m "Organize MCEN90028 robotics coursework and experiments"
git remote add origin <YOUR_GITHUB_REPOSITORY_URL>
git push -u origin main
```

Replace the placeholder with the actual repository URL. The configured remote is https://github.com/Jason-Zhang-Pro/Robotic-Arm.git.
