function mth = MTH(d, lambda, q)
    m_rot = expm(OPC(lambda)*q);
    mth = [m_rot d'; 0 0 0 1];
end
