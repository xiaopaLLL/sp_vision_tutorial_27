# 项目理解报告

请尽量使用自己的语言回答以下问题。可以引用少量关键代码或伪代码，但不要只粘贴实现。
完成一节后删除该节末尾的待填写标记；本地检查会拒绝仍有未完成章节的报告。

## 1. 图像生命周期与所有权

解释本项目中图像源为什么会复用缓冲区，以及 `cv::Mat` 的普通复制对底层像素数据
意味着什么。说明你的修改让一个 `Frame` 在进入队列后拥有什么，并解释为何后续读取
不会再改变它。

### ANSWER(report)：

在ImageSequenceSource::next方法中，拷贝buffer_到frame.image时采用的是frame.image = buffer_;，由于该两个变量都是cv::Mat类型，在=赋值时都采用浅拷贝方法，解决方法很简单，把frame.image = buffer_;改成frame.image = buffer_.clone();深拷贝即可，修改后frame进入队列后拥有cv::Mat image,int id,std::uint64_t checksum，这些全都是深拷贝后的副本值，不是浅拷贝下的header

在Pipeline::profucerLoop方法中，由于该方法被while循环调用，于是每次浅拷贝实则是让frame.image指向了buffer_中的内容，buffer_每被修改一次，所有的frame.image同样受影响，这导致前0-18帧process时，frame.image已经指向第19帧的buffer_，这对应了为什么初次编译时只output了019.jpg

这里十分佩服frame.hpp里的checksum设计，它为frame计算了哈希值存到了成员expected_checksum中，且该成员是i64类型，并不会在=赋值时受到浅拷贝影响，因此通过对比image的哈希值和expected_sum来确定对图像处理时是否受浅拷贝篡改，并在logLine方法中输出（实际workerLoop也是这么干的），觉得这个设计非常精妙，由衷佩服

在Pipeline::producerLoop方法中的提问"What's the best way to write this?"的写法应是queue_.push(std::move(frame));，其实就是把一次浅拷贝的开销变为一次移动的开销，因为后续的next()调用时会覆盖此处被move走的frame的内容，所以此处的写法没有问题

## 2. 并发处理与恰好一次

结合 `BlockingQueue` 的 `push`、`pop` 和 `close` 行为，解释多个 worker 如何分工。
为什么你的实现既不会漏掉已经入队的帧，也不会重复处理同一帧？输入耗尽时，正在等待
以及仍在处理数据的 worker 分别会怎样？

### ANSWER(report)：

总体的BlockingQueue的运行逻辑是：producerLoop调push，workerLoop调pop，在producerLoop结束的时候调一次close

注意到pop中有一行ready_.wait()非常有意思，其中ready_是一个条件变量，在这行wait方法中让该线程绑定了ready_，仅当close或push时可被唤醒。

在此处的跳过wait条件大有玄机：如果closed_==false，那么说明必然queue_.empty()==false，在此时被唤醒线程进入return true分支，线程可以正常执行；如果closed_==true，那么queue_.empty()==false时线程正常执行，queue_.empty()==true时线程进入return false分支，workerLoop中while循环退出

不会漏帧或重复处理的原因很简单，就是因为每个pop都共用一个queue_的mutex，线程间无法互相影响。

输入耗尽这个有点说法，一种情况是producer的速度跟不上worker（应该对应的是初期），这导致push过程仍在进行queue_为空，此时closed_=false，因此此时调用pop的worker线程进入wait状态，push完成后notify_one被调用，此刻queue_中立刻有一个元素，至少可以供一个wait中的worker执行pop，所以仅唤醒一个wait中的线程，处理数据的worker正常执行，处理完后要么producer已经将新的frame push到队列跳过wait，要么队列还是空的进入wait等待notify_one或notify_all唤醒

另一种情况是producerLoop结束，但workerLoop还在进行（对应的是后期），结束后close()被调用后queue_.close_变为true，同时执行了notify_all，所有worker线程全被唤醒，并且由于closed_==true，它们再也不会进入wait状态，直到queue_变为空时queue_.pop()返回false，workerLoop退出

## 3. 共享统计数据

指出哪些线程会读写 `Statistics`。解释原实现中的竞争为什么可能导致错误结果，并说明
你的同步方案提供了什么保证。还应说明取得快照时为什么是安全的。

### ANSWER(report)：

producer线程写入statistics_.produced，worker线程写入statistics_.processed,statistics_.corrupted和statistics_.saved，main线程对这4个数据读取

在仅完成frame_source.cpp的修改时，最终的结果是produced=20,processed和saved小于20(且有时不相等),corrupted=0(这代表第一问没做错)

在多worker共用一个线程池时，由于deliberatelySlowIncrement的设计（这个设计也好聪明可以方放大线程影响的概率），多个线程可能会先一起执行const int old = value;，然后再执行value = old + 1;，这导致不同线程下用了同一个old，就使value少了几次自增，而saved和processed的这个次数可能不一样，就导致了他俩可能不相等

解决方法就是，在每处deliberatelySlowIncrement都套一个锁，这样就不会影响了。此外因为要保证读取时也安全，所以要在snapshot那边也要加一个mutex

加mutex的时候一开始想在cpp里弄一个全局变量，但是ai不推荐我这么干说建议加到private成员里（原因是每个实例应该分开管理不应该所有实例都用一把锁），于是去加了之后发现编译过不去，最后发现居然是hpp里没有include mutex导致的。改完之后发现还过不去，因为snapshot是const方法不能直接使用mutex，解决方法居然是在mutex声明前加mutable。反正我写完整个人都无语住了，感觉是出题人故意设的坑我全踩到了一遍....

## 4. 线程关闭协议

分别描述以下两条路径中的事件顺序，并解释为什么不会发生 `std::terminate`、悬空访问
或永久等待：

1. 调用者执行 `start()` 后显式调用 `wait()`；
2. 调用者执行 `start()` 后不调用 `wait()`，直接让 `Pipeline` 析构。

如果你的实现允许某个生命周期方法被重复调用，也请说明其行为；如果不允许，请说明前置条件。

### ANSWER(report)：

在修改析构函数之前：

1.main线程中调用start()后producer和workers线程运行，之后main线程中调用wait()，producer线程和worker线程依次运行join，经过工作后，producer先结束，queue_清空后worker依次结束，此时所有join的线程等待完毕，main线程return 0，并析构pipeline

2.main线程中调用start()后如前，不调用wait()，main线程直接return 0，在此处析构pipeline时，由于pipeline的各线程的joinable()==true，因此造成std::terminate

针对未显式调用wait()的报错问题，实现时只需要在对象析构前将各线程join一遍即可

本来写的代码是
```cpp
producer_.join();
for(auto &worker : workers_) worker.join();
```
但是这么写发现在未显式调用wait()时不报错，显式调用wait()反而报错，发现原来是析构时无条件将线程们join了一遍，而先前调用的wait()已经join过一遍了，join一个已经join过的线程就会导致报错，只有在joinable()==true的时候才可以join（对应的是没join过的时候），因此将代码修改成
```cpp
if(producer_.joinable()) producer_.join();
for(auto &worker : workers_) {if(worker.joinable()) worker.join();}
```
这样就避免了重复join和不显式调用wait()导致的没join的问题，但是我一看这不就和wait()的内容一模一样吗，于是就开开心心的把析构函数里的东西删光，只留下一行漂亮的wait();了

修改后的话，就变成这样：

1.start()->wait()->~Pipeline()，析构时空转一轮wait()，join了0个线程

2.start()->~Pipeline()，析构时wait()了，所有线程被join

不会永久等待的问题在BlockingQueue中就解决了，在close时会notify_all一遍，在那之后线程就再也不会进入等待状态了

至于为什么不会悬空访问呢，因为析构pipeline会在pipeline的成员之前，所以调用析构函数里的wait()的时候pipeline的成员queue_和statistics_还存活，所以等那些进程的时候可以正常访问