function [blk] = vector_from_workspace_init(blk,varargin)

  log_group = 'vector_from_workspace_init_debug';
  % clog('entering vector_from_workspace_init', {log_group, 'trace'});
  
  % Set default vararg values.
  % reg_retiming is not an actual parameter of this block, but it is included
  % in defaults so that same_state will return false for blocks drawn prior to
  % adding reg_retiming='on' to some of the underlying Delay blocks.
  defaults = { ...
    'n_in', 0, ...
    'n_bits', 0,...
    'bin_pt',0,...
    'arith_type',0,...
    'var_pfx','0'...
  };  
  
  check_mask_type(blk, 'vector_from_workspace');

  if same_state(blk, 'defaults', defaults, varargin{:}), return, end
  munge_block(blk, varargin{:});

  n_in                  = get_var('n_in', 'defaults', defaults, varargin{:});
  n_bits                = get_var('n_bits', 'defaults', defaults, varargin{:});
  bin_pt                = get_var('bin_pt','defaults',defaults,varargin{:});
  arith_type            = get_var('arith_type','defaults',varargin{:});
  var_pfx               = get_var('var_pfx','defaults',varargin{:});

  %%%%%%%%%%%%%%%%%%%%%%
  % parameter checking %
  %%%%%%%%%%%%%%%%%%%%%%

  % redefine arith_type
  switch(arith_type)
      case 0
          arith_type = 'Unsigned';
      case 1
          arith_type = 'Signed  (2''s comp)';
      case 2
          arith_type = 'Boolean';
  end
  % delete lines
  delete_lines(blk);
  
  xpos = 260; xinc = 200;
  ypos = 172; yinc = 80;

  fromws_w = 100; fromws_d = 28;
  gin_w = 50; gin_d = 28;
  port_w = 30; port_d = 14;
  buse_w = 70; buse_d = n_in*20;
  drss_w = 50; drss_d = 50;
  
  %%%
  % input
  %%%
  xpos_tmp = xpos;
  ypos_tmp = ypos;
  for ii=1:n_in
      reuse_block(blk,sprintf('v%d',n_in-ii+1),'built-in/FromWorkspace',...
          'Position',[xpos_tmp-fromws_w/2 ypos_tmp-fromws_d/2 xpos_tmp+fromws_w/2 ypos_tmp+fromws_d/2],...
          'VariableName',sprintf('%s_%d',var_pfx,n_in-ii+1));
      ypos_tmp = ypos_tmp + yinc;
  end
  
  %%%
  % gate
  %%%
  xpos_tmp = xpos_tmp + xinc;
  ypos_tmp = ypos;
  for ii=1:n_in
      reuse_block(blk,sprintf('g%d',n_in-ii+1),'xbsIndex_r4/Gateway In',...
          'Position',[xpos_tmp-gin_w/2 ypos_tmp-gin_d/2 xpos_tmp+gin_w/2 ypos_tmp+gin_d/2],...
          'n_bits',num2str(n_bits),'bin_pt',num2str(bin_pt),'arith_type',arith_type);
      ypos_tmp = ypos_tmp + yinc;
      add_line(blk,sprintf('v%d/1',n_in-ii+1),sprintf('g%d/1',n_in-ii+1));
  end
  
  %%%
  % bus expand
  %%%
  xpos_tmp = xpos_tmp + xinc;
  ypos_tmp = ypos;
  reuse_block(blk,'bus_create','casper_library_flow_control/bus_create',...
      'Position',[xpos_tmp-buse_w/2 ypos_tmp-buse_d/2 xpos_tmp+buse_w/2 ypos_tmp+buse_w/2],...
      'inputNum',num2str(n_in));
  for ii=1:n_in
      add_line(blk,sprintf('g%d/1',n_in-ii+1),sprintf('bus_create/%d',ii));
  end
  
  %%%
  % port
  %%%
  xpos_tmp = xpos_tmp + xinc;
  ypos_tmp = ypos + buse_d/2; 
  reuse_block(blk,'from_ws','built-in/outport',...
      'Port', '1', 'Position', [xpos_tmp-port_w/2 ypos_tmp-port_d/2 xpos_tmp+port_w/2 ypos_tmp+port_d/2]);
  add_line(blk,'bus_create/1','from_ws/1');
  
  % When finished drawing blocks and lines, remove all unused blocks.
  clean_blocks(blk);
  
  % add a disregard subsystem block
  xpos_tmp = xpos_tmp - 2*xinc;
  ypos_tmp = ypos - yinc;
  reuse_block(blk,'disregard','xbsIndex_r4/Disregard Subsystem',...
      'Position',[xpos_tmp-drss_w/2 ypos_tmp-drss_d/2 xpos_tmp+drss_w/2 ypos_tmp+drss_d/2]);

  save_state(blk, 'defaults', defaults, varargin{:});  % Save and back-populate mask parameter values

  % clog('exiting vector_from_workspace_init', {log_group, 'trace'});

end %function vector_from_workspace_init

