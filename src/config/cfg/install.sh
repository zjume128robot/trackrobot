#back and copy user config
cp share/config/cfg/.bashrc.local.install ~/.bashrc.local
chmod -R +r ~/install/lib/
chmod -R +x ~/install/lib/
chmod +x ~/install/setup.bash
chmod +x ~/install/_setup_util.py
cd share/config/cfg/
chmod +x cfg.sh
./cfg.sh
cd ~/install
bash
