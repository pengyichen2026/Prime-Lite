sudo apt --fix-broken install
sudo apt update
sudo apt install python3-pkg-resources=68.1.2-2ubuntu1.2 python3-setuptools=68.1.2-2ubuntu1.2
sudo apt install python3-pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
sudo apt install python3-tk
pip install robstride_dynamics --break-system-packages
pip install loop_rate_limiters --break-system-packages
