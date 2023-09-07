/* 
////////////////  Developed By Hamid.Memar (2023-Revised)  ////////////////
///////////////   Licensed Under MIT Terms And Arguments   ////////////////
*/

try
(
	rollout mxmeshUI "MXMesh 2.4 (BETA)" width:185 height:755
	(
		-- User Interface
		GroupBox 'p1' ".:: Operations ::." pos:[6,7] width:170 height:158 align:#left
		GroupBox 'p2' ".:: Settings ::." pos:[6,248] width:170 height:207 align:#left
		GroupBox 'p3' ".:: Tools  ::." pos:[6,170] width:170 height:74 align:#left
		listbox 'chckPts' ".:: Checkpoints ::." pos:[6,458] width:170 height:16 align:#left
		button 'cacheSel' "Cache Selection" pos:[15,27] width:150 height:20 align:#left
		button 'restoreFrom' "Restore From File..." pos:[15,49] width:150 height:20 align:#left
		button 'addChk' "Add Checkpoint" pos:[16,71] width:150 height:20 enabled:true align:#left
		button 'restoreChk' "Restore Checkpoint" pos:[16,93] width:150 height:20 align:#left
		button 'restoreAll' "Restore All" pos:[16,116] width:150 height:20 align:#left
		button 'selPath' "Set Cache Path..." pos:[16,267] width:150 height:20 align:#left
		button 'prugeCollect' "Purge and Collect" pos:[16,139] width:150 height:20 align:#left
		button 'restore' "Restore" pos:[5,695] width:84 height:23 align:#left
		button 'del' "Delete" pos:[94,695] width:84 height:23 align:#left
		button 'copyMesh' "Copy Mesh" pos:[16,192] width:150 height:20 align:#left enabled:false
		button 'pasteMesh' "Paste Mesh" pos:[16,216] width:150 height:20 align:#left enabled:false
		dropdownList 'buffMod' "Cache Buffering Mode" pos:[16,295] width:151 height:40 items:#("In-Memory (Faster)", "On-Disk (Optimized)") selection:1 align:#left
		dropdownList 'restoreMod' "Restore Mode" pos:[16,339] width:151 height:40 items:#("Single Thread", "Multi Threaded") selection:2 align:#left
		dropdownList 'compMod' "Compression Mode" pos:[16,384] width:151 height:40 items:#("Faster", "Better (Smaller)") selection:2 align:#left
		checkbox 'dbgMode' "Debug Mode" pos:[16,432] width:97 height:15 align:#left
		label 'dev' "By MemarDesign™ LLC." pos:[8,729] width:167 height:18 align:#left
		
		-- Functions
		fn refreshList = 
		(
			chckPts.items = #()
			mxoPackages = getFiles (MXMesh.GetCachePath() + "\*.mxo*")
			for mxo in mxoPackages do chckPts.items = Append chckPts.items (getFilenameFile mxo)
		)
		
		fn restoreByName mxoName = 
		(
			undo "MXMesh::RestoreMesh" on 
			(
				if(mxoName == undefined ) do return false
				MXMesh.Restore (MXMesh.GetCachePath() + "\\" + mxoName + ".mxo")
			)
		)
		
		fn restoreMeshByName mxoName = 
		(
			if(mxoName == undefined or $ == undefined) do return false
			MXMesh.RestoreMesh (MXMesh.GetCachePath() + "\\" + mxoName + ".mxo") $
		)
		
		-- Events
		on mxmeshUI open do
		(
			refreshList()
		)
		
		on cacheSel pressed do
		(
			for obj in selection do MXMesh.Cache obj
			refreshList()
		)
		
		on restoreFrom pressed do
		(
			mxo = getOpenFileName types:"Max Mesh Object (Zipped)(*.mxo)|*.mxo"
			if mxo != undefined do MXMesh.Restore mxo
		)
		
		on addChk pressed do
		(
			if $ != undefined do
			(
				MXMesh.Checkpoint $
				refreshList()
			)
		)
		
		on restoreChk pressed do
		(
			restoreMeshByName(chckPts.selected)
		)
		
		on restoreAll pressed do
		(
			mxoPackages = getFiles (mxmesh.getcachepath() + "\*.mxo*")
			for mxo in mxoPackages do mxmesh.restore(mxo)
		)
		
		on selPath pressed do
		(
			get_path = getSavepath()
			if get_path != undefined do 
			(
				MXMesh.SetCachePath get_path
				refreshList()
			)
		)
		
		on prugeCollect pressed do
		(
			try(MXMesh.Purge()) catch(MessageBox "Make sure you have full access to cache path." title:"Access Error")
			refreshList()
		)
		
		on restore pressed do
		(
			restoreByName(chckPts.selected)
		)
		
		on del pressed do
		(
			sure = QueryBox "Are you sure?" title:"Warning"
			if(sure) do 
			(
				deletefile (MXMesh.GetCachePath() + "\\" + chckPts.selected + ".mxo")
				refreshList()
			)
		)
		
		on chckPts doubleClicked sel do
		(
			restoreByName(chckPts.items[sel])
		)
		
		on buffMod selected sel do
		(
			if(sel == 1) do MXMesh.SetCacheBufferingMode #memory
			if(sel == 2) do MXMesh.SetCacheBufferingMode #disk
		)
		
		on restoreMod selected sel do
		(
			if(sel == 1) do MXMesh.SetRestoreMode #single
			if(sel == 2) do MXMesh.SetRestoreMode #multi
		)
		
		on compMod selected sel do
		(
			if(sel == 1) do MXMesh.SetCompressionMode #faster
			if(sel == 2) do MXMesh.SetCompressionMode #better
		)
		
		on copyMesh pressed do 
		(
			if selection.count == 1 do
			(
				MXMesh.CacheToMemory $
			)
		)
		
		on pasteMesh pressed do 
		(
			if selection.count == 1 do
			(
				MXMesh.CacheToMemory $
			)
		)
		
		on dbgMode changed state do
		(
			MXMesh.SetDebugMode state
		)
	)

	createdialog mxmeshUI
) catch ( messageBox ("Fatal Error : " + getCurrentException()) title:"MXMesh Error" )