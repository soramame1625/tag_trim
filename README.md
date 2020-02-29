
# tag_trim　　

# DEMO
   

```bash
git cllone https://github.com/soramame1625/tag_trim.git
```
- トリミング　

    **trimm.launch**
- マスキング　

    **mask.launch**

# 実行方法
- ワークススペースに当パッケージをクローンします。
   ```
   cd catkin_ws/src
   git clone https://github.com/soramame1625/tag_trim.git
   ```
 - AprilTagsパッケージをクローンします。
   通常のAprilTagsでは実行できないので以下のものを使用してください。
   ```
   git clone https://github.com/soramame1625/apriltag_ros.git
   ```
   
```bash
git clone https://github.com/hoge/~
cd examples
python demo.py
```

    ---
    4行目　 自分が使用するカメラのパスを指定
    ```
    <param name="video_device" value="/dev/video1" />　
