
## 1. Docker Installation Documentation:

You can follow the official Docker installation documentation for Ubuntu by visiting the following link:
[Docker Install Guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

### Step-by-Step Installation:

1. Update your package list:

   ```bash
   sudo apt-get update
   ```

2. Install the required packages:

   ```bash
   sudo apt-get install ca-certificates curl
   ```

3. Add Docker's official GPG key:

   ```bash
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc
   ```

4. Add the Docker repository to your Apt sources:

   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   ```

5. Update your package list again:

   ```bash
   sudo apt-get update
   ```

6. Install Docker:

   ```bash
   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```

7. Verify Docker installation by running the hello-world container:

   ```bash
   sudo docker run hello-world
   ```

## 2. Docker Post-Installation Steps:

After installing Docker, you can follow these post-installation steps:
[Docker Post-Install Guide](https://docs.docker.com/engine/install/linux-postinstall/)

1. Create the Docker group:

   ```bash
   sudo groupadd docker
   ```

2. Add your user to the Docker group:

   ```bash
   sudo usermod -aG docker $USER
   ```

3. Log out and log back in for the changes to take effect (on WSL simply do `wsl --shutdown` and restart the WSL terminal).

4. Start a new shell session:

   ```bash
   newgrp docker
   ```

5. Verify Docker is working without `sudo`:

   ```bash
   docker run hello-world
   ```

6. Enable Docker services to start on boot:

   ```bash
   sudo systemctl enable docker.service
   sudo systemctl enable containerd.service
   ```
 7. Add the following line to your `~/.bashrc` to allow GUI applications to run from within Docker containers:

   Copy paste this command to your terminal:
   ```bash
   xhost +local:docker
   ```

   To make this change permanent, add the following line to your `~/.bashrc` file:

   ```bash
   echo "xhost +local:docker" >> ~/.bashrc
   ```


## 3. Nvidia Container Toolkit

If your device is equipped with an NVIDIA GPU and you want to use it within Docker containers, you need to install the NVIDIA container toolkit. This allows Docker containers to access the GPU for tasks such as machine learning, rendering, and other GPU-accelerated workloads.

Full documentation: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

Add the NVIDIA container toolkit repository:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Update packages and install the toolkit:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

Then restart your computer.