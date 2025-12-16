---
layout: page
title: Interfaces 
permalink: /start/interface
nav: 3 
parent: Getting started 
has_children: true
math: mathjax3
has_toc: true 
---
To illustrate the different APIs we consider the simple QP

$$\begin{aligned}
&\underset{x_1, x_2}{\text{minimize}}&& \frac{1}{2} (x_1^2 + x_2^2) + x_1 + x_2\\
&\text{subject to}  &&-1\leq x_1\leq 1 \\
& &&-2\leq x_2\leq 2 \\
& &&-3\leq x_1 + x_2\leq 3 \\
& &&-4\leq x_1 - x_2\leq 4, \\
\end{aligned}$$

which can be put in the form 

$$\begin{aligned}
&\underset{x}{\text{minimize}}&& \frac{1}{2} x^T H x + f^T x\\
&\text{subject to} && l\:\: \leq \:x \:\:\leq u, \\
& && b_l \leq A x \leq b_u, \\
\end{aligned}$$

with

$$
H = \begin{pmatrix} 1 & 0 \\ 0 & 1 \end{pmatrix}, \quad f = \begin{pmatrix} 1 \\ 1 \end{pmatrix},\quad  
$$

$$
u =\begin{pmatrix} 1 \\ 2 \end{pmatrix}, \quad
l =-\begin{pmatrix} 1 \\ 2 \end{pmatrix}, \quad
A = \begin{pmatrix} 1 & 1 \\ 1 & -1 \end{pmatrix},\quad 
b_u =\begin{pmatrix} 3 \\ 4 \end{pmatrix},\quad
b_l =-\begin{pmatrix} 3 \\ 4 \end{pmatrix}
$$
