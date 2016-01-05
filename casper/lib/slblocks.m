function blkStruct = slblocks
		% This function specifies that the library should appear
		% in the Library Browser
		% and be cached in the browser repository

		Browser.Library = 'r2daq_blocks';
		% 'mylib' is the name of the library

		Browser.Name = 'ROACH2 Data Acquisition System for Project 8';
		% 'My Library' is the library name that appears in the Library Browser

		blkStruct.Browser = Browser;