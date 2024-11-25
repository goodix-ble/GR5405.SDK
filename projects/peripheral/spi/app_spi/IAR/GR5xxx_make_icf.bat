:: Genenate the preprocessing file
iccarm -D __GR5405__ --preprocess=n preprocess_icf.i -I .\..\src\config\ -I .\..\..\..\..\..\platform\soc\include  .\flash_icf_config.c --cpu Cortex-M4

python .\make_app_dependent_icf.py -i .\preprocess_icf.i -o .\gr5405.icf

del .\preprocess_icf.i
