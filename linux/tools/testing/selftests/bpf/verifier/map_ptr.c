{
	"bpf_map_ptr: read with negative offset rejected",
	.insns = {
	BPF_MOV64_REG(BPF_REG_1, BPF_REG_10),
	BPF_LD_MAP_FD(BPF_REG_1, 0),
	BPF_LDX_MEM(BPF_DW, BPF_REG_6, BPF_REG_1, -8),
	BPF_MOV64_IMM(BPF_REG_0, 1),
	BPF_EXIT_INSN(),
	},
	.fixup_map_array_48b = { 1 },
	.result_unpriv = REJECT,
	.errstr_unpriv = "bpf_array access is allowed only to CAP_PERFMON and CAP_SYS_ADMIN",
	.result = REJECT,
	.errstr = "R1 is bpf_array invalid negative access: off=-8",
},
{
	"bpf_map_ptr: write rejected",
	.insns = {
	BPF_ST_MEM(BPF_DW, BPF_REG_10, -8, 0),
	BPF_MOV64_REG(BPF_REG_2, BPF_REG_10),
	BPF_ALU64_IMM(BPF_ADD, BPF_REG_2, -8),
	BPF_LD_MAP_FD(BPF_REG_1, 0),
	BPF_STX_MEM(BPF_DW, BPF_REG_1, BPF_REG_2, 0),
	BPF_MOV64_IMM(BPF_REG_0, 1),
	BPF_EXIT_INSN(),
	},
	.fixup_map_array_48b = { 3 },
	.result_unpriv = REJECT,
	.errstr_unpriv = "bpf_array access is allowed only to CAP_PERFMON and CAP_SYS_ADMIN",
	.result = REJECT,
	.errstr = "only read from bpf_array is supported",
},
{
	"bpf_map_ptr: read non-existent field rejected",
	.insns = {
	BPF_MOV64_IMM(BPF_REG_6, 0),
	BPF_LD_MAP_FD(BPF_REG_1, 0),
	BPF_LDX_MEM(BPF_W, BPF_REG_6, BPF_REG_1, 1),
	BPF_MOV64_IMM(BPF_REG_0, 1),
	BPF_EXIT_INSN(),
	},
	.fixup_map_array_48b = { 1 },
	.result_unpriv = REJECT,
	.errstr_unpriv = "bpf_array access is allowed only to CAP_PERFMON and CAP_SYS_ADMIN",
	.result = REJECT,
	.errstr = "cannot access ptr member ops with moff 0 in struct bpf_map with off 1 size 4",
	.flags = F_NEEDS_EFFICIENT_UNALIGNED_ACCESS,
},
{
	"bpf_map_ptr: read ops field accepted",
	.insns = {
	BPF_MOV64_IMM(BPF_REG_6, 0),
	BPF_LD_MAP_FD(BPF_REG_1, 0),
	BPF_LDX_MEM(BPF_DW, BPF_REG_6, BPF_REG_1, 0),
	BPF_MOV64_IMM(BPF_REG_0, 1),
	BPF_EXIT_INSN(),
	},
	.fixup_map_array_48b = { 1 },
	.result_unpriv = REJECT,
	.errstr_unpriv = "bpf_array access is allowed only to CAP_PERFMON and CAP_SYS_ADMIN",
	.result = ACCEPT,
	.retval = 1,
},
{
	"bpf_map_ptr: r = 0, map_ptr = map_ptr + r",
	.insns = {
	BPF_ST_MEM(BPF_DW, BPF_REG_10, -8, 0),
	BPF_MOV64_REG(BPF_REG_2, BPF_REG_10),
	BPF_ALU64_IMM(BPF_ADD, BPF_REG_2, -8),
	BPF_MOV64_IMM(BPF_REG_0, 0),
	BPF_LD_MAP_FD(BPF_REG_1, 0),
	BPF_ALU64_REG(BPF_ADD, BPF_REG_1, BPF_REG_0),
	BPF_RAW_INSN(BPF_JMP | BPF_CALL, 0, 0, 0, BPF_FUNC_map_lookup_elem),
	BPF_MOV64_IMM(BPF_REG_0, 0),
	BPF_EXIT_INSN(),
	},
	.fixup_map_hash_16b = { 4 },
	.result_unpriv = REJECT,
	.errstr_unpriv = "R1 has pointer with unsupported alu operation",
	.result = ACCEPT,
},
{
	"bpf_map_ptr: r = 0, r = r + map_ptr",
	.insns = {
	BPF_ST_MEM(BPF_DW, BPF_REG_10, -8, 0),
	BPF_MOV64_REG(BPF_REG_2, BPF_REG_10),
	BPF_ALU64_IMM(BPF_ADD, BPF_REG_2, -8),
	BPF_MOV64_IMM(BPF_REG_1, 0),
	BPF_LD_MAP_FD(BPF_REG_0, 0),
	BPF_ALU64_REG(BPF_ADD, BPF_REG_1, BPF_REG_0),
	BPF_RAW_INSN(BPF_JMP | BPF_CALL, 0, 0, 0, BPF_FUNC_map_lookup_elem),
	BPF_MOV64_IMM(BPF_REG_0, 0),
	BPF_EXIT_INSN(),
	},
	.fixup_map_hash_16b = { 4 },
	.result_unpriv = REJECT,
	.errstr_unpriv = "R0 has pointer with unsupported alu operation",
	.result = ACCEPT,
},
