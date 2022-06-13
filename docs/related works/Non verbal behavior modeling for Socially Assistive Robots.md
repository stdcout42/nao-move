## Non verbal behavior modeling for Socially Assistive Robots

- Socially assistive robotics (SAR)

  <img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161452950.png" alt="image-20220608161452950" style="zoom:80%;" />

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161518905.png" alt="image-20220608161518905" style="zoom:80%;" />

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161537482.png" alt="image-20220608161537482" style="zoom:80%;" />

- Focus of research

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161721240.png" alt="image-20220608161721240" style="zoom:80%;" />

- Demand for data-driven robot behavior models

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161800702.png" alt="image-20220608161800702" style="zoom:80%;" />

- Model

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161826189.png" alt="image-20220608161826189" style="zoom:80%;" />

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608161837298.png" alt="image-20220608161837298" style="zoom:50%;" />

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608163042159.png" alt="image-20220608163042159" style="zoom:80%;" />

​				This is similar to our multi-modal system in that we also collect data (hand gestures) 				from users and train a network to recognize the gestures. 

- Data collection

  <img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608163414826.png" alt="image-20220608163414826" style="zoom:80%;" />

​							This data was then manually annotated (for non verbal behavioral features)

- Classification

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608163730968.png" alt="image-20220608163730968" style="zoom:80%;" />

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608163753581.png" alt="image-20220608163753581" style="zoom:80%;" />

- Robot behavior generation

<img src="/home/sam/snap/typora/57/.config/Typora/typora-user-images/image-20220608165053087.png" alt="image-20220608165053087" style="zoom:80%;" />

This paper is similar to ours in the sense that we also try to predict context (but also commands) based on non-verbal input (gestures). Where it differs is of course the type of none-verbal input (they use gaze, gestures, context and objects) as well as they attempt as to generate robot behavior given a context (bi-directional)