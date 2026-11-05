Mini tank: assembly + drive animation, 720 frames
=================================================
Format : JPEG, 1920x1080, numbered tank_0000.jpg ... tank_0719.jpg
Rate   : 24 frames per second  ->  30 seconds

Timeline
  0.0 -  3.0 s   all parts spread out, camera circling
  3.0 -  6.5 s   motors, battery and motor caps drop into the hull
  6.5 - 10.0 s   sprockets, road wheels, idlers, sleeves, screws and nuts slide in
 10.0 - 13.5 s   both tracks move in, then start running
 13.5 - 16.5 s   deck with servo and camera head lowers on
 16.5 - 23.0 s   drives forward, turns 90 degrees on the spot, drives on
 23.0 - 27.0 s   camera moves in; camera head pans left and right
 27.0 - 30.0 s   camera pulls back around the parked tank

Importing as a video
  DaVinci Resolve : Media pool > import the folder; it detects the numbered sequence as one clip.
                    Set the clip/timeline frame rate to 24 fps.
  Premiere Pro    : File > Import > select tank_0000.jpg > tick "Image Sequence".
                    Interpret footage at 24 fps.
  Final Cut Pro   : import all frames, select them, set still-image duration to 1 frame
                    (or use Compressor / an image-sequence tool).
  iMovie / phone editors : these don't import sequences well; convert first, e.g. with ffmpeg:
      ffmpeg -framerate 24 -i tank_%04d.jpg -c:v libx264 -pix_fmt yuv420p -crf 18 tank.mp4
