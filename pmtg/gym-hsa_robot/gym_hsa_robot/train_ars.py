'''
Quote from:
https://towardsdatascience.com/introduction-to-augmented-random-search-d8d7b55309bd

Let 𝛎 a positive constant < 1
Let 𝝰 be the learning rate
Let N the number of perturbations
Let 𝜃 a (p x n) matrix representing the parameters of the policy 𝜋
Let 𝜹i a (p x n) matrix representing the ith perturbation
1. While end condition not satisfied do:
2. Generate N perturbations 𝜹 from a normal distribution
3. Normalize 𝜋i+ = (𝜃+𝛎𝜹i)ᵀx and 𝜋i- = (𝜃-𝛎𝜹i)ᵀx for i = 1 to N
4. Generate 2N episodes and their 2N rewards using 𝜋i+ and 𝜋i- and collect the rewards ri+ and ri-
5. Sort all 𝜹 by max(ri+, ri-)
6. Update 𝜃 = 𝜃 + (𝝰/(b*𝞼ᵣ)) Σ(ri+ - ri-)𝜹i (where i = 1 to b)
7. End While


Policy must have:
Robot state (x,y,velocity)
TG Params (width, height)
Phase of each foot (perhaps?)
Return: TG Params, 8 parameters for leg modifications

    Dimensions: 6 x 10(?)

TG must have:
Input: width, height, timestep (opt)
Find ellipse with width, height, at certain timestep
Return: theta, eps (leg position)



'''
