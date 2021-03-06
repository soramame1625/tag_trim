
# tag_trim　　
AprilTagsと同時に起動することでAprilTagsの認識速度を向上させることが可能です。

# DEMO
![hgldg-vc9k5](https://user-images.githubusercontent.com/39610790/75664229-792c3e00-5cb5-11ea-9348-3f9c4bdb0aef.gif)

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
- .bashrcに以下を追加します。
  ```
  source ~/catkin_ws/devel/setup.bash
  ```
- マスキングとトリミングはそれぞれ以下で実行可能です。
  ```
  roslaunch tags_test mask.launch   //マスキング
  roslaunch tags_test trim.launch   //トリミング
  ```
- 実行前にそれぞれのlaunchファイルの4行目で使用するカメラのパスを指定する必要があります。
  ```
  <param name="video_device" value="/dev/video1" />　
  ```
