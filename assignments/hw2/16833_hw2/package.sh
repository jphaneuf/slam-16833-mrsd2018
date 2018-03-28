pandoc jphaneuf.md -o jphaneuf.pdf

mkdir jphaneuf_hw2
cp -r EKF_slam jphaneuf_hw2
cp jphaneuf.pdf jphaneuf_hw2

zip -r jphaneuf_hw2.zip jphaneuf_hw2
