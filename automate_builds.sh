# version -1

# #!/bin/bash
# set -e
# trap 'echo -e "\e[31m❌ Error occurred. Exiting.\e[0m"' ERR

# # 🎨 Colors
# GREEN="\e[32m"
# RED="\e[31m"
# YELLOW="\e[33m"
# RESET="\e[0m"

# # 📋 Available options
# BOARDS=("zb_tv_h743zi" "zb_tv_h723zi" "zb_bv_l433rc" "zb_bv_l151c8" "zb_acrux_vp1_0_revb")
# VARIANTS=("ACTIPOD_4L" "ACTIPOD_4L_HIGHLY" "ACTIPOD_4L_HIGHLY_4G" "ACTIPOD_15L_4G" "ACTIPOD_15L_HYBRID" "ACTIPOD_4L_444" "ACTIPOD_15L")
# # 📥 Get board choices
# echo -e "${YELLOW}Available Boards:${RESET}"
# # for i in "${!BOARDS[@]}"; do
# #     echo "$((i+1)). ${BOARDS[$i]}"
# # done
# # read -p "👉 Enter board numbers (comma-separated): " board_input

# # # 🧮 Convert input to selected boards
# # IFS=',' read -ra BOARD_INDICES <<< "$board_input"
# # SELECTED_BOARDS=()
# # for index in "${BOARD_INDICES[@]}"; do
# #     SELECTED_BOARDS+=("${BOARDS[$((index-1))]}")
# # done

# # # 📥 Get variant choices
# # echo -e "${YELLOW}Available Variants:${RESET}"
# # for i in "${!VARIANTS[@]}"; do
# #     echo "$((i+1)). ${VARIANTS[$i]}"
# # done
# # read -p "👉 Enter variant numbers (comma-separated): " variant_input

# # IFS=',' read -ra VARIANT_INDICES <<< "$variant_input"
# # SELECTED_VARIANTS=()
# # for index in "${VARIANT_INDICES[@]}"; do
# #     SELECTED_VARIANTS+=("${VARIANTS[$((index-1))]}")
# # done

# # # 🏗️ Create build_custom folder
# # BUILD_DIR="build_custom"
# # mkdir -p "$BUILD_DIR"

# # # 🧾 Save Git commit info
# # echo -e "${YELLOW}🔍 Saving Git commit info...${RESET}"
# # git log -1 > "$BUILD_DIR/commit_info.txt"

# # # 🔁 Build loop
# # for board in "${SELECTED_BOARDS[@]}"; do
# #     for variant in "${SELECTED_VARIANTS[@]}"; do
# #         OUT_DIR="$BUILD_DIR/$board/$variant"
# #         echo -e "${YELLOW}🔨 Building for $board [$variant]...${RESET}"
# #         mkdir -p "$OUT_DIR"

# #         # ✨ Replace below with your real build command
        
# #         export VERSION=D4_$(date +%d-%m-%y_%H-%M-%S)
# #         west build -p -c -b "$board" zb_actipod_app -d "$OUT_DIR"  -- -DCONFIG_"$variant"=y -DCONFIG_APP_SYS_VERSION=\"$(echo $VERSION)\" -DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE=\"bootloader/mcuboot/zb-ed25519.pem\" > "$OUT_DIR/build_log.txt" 2>&1




# #         echo -e "${GREEN}✅ Done: $board [$variant]${RESET}"
# #     done
# # done

# # echo -e "${GREEN}🎉 All builds complete! Check the '$BUILD_DIR' folder.${RESET}"

# #version2

# #!/bin/bash
# set -e
# trap 'echo -e "\e[31m❌ Error occurred. Exiting.\e[0m"' ERR

# # 🎨 Colors
# GREEN="\e[32m"
# RED="\e[31m"
# YELLOW="\e[33m"
# RESET="\e[0m"

# # 📋 Available options
# BOARDS=("zb_tv_h743zi" "zb_tv_h723zi" "zb_bv_l433rc" "zb_bv_l151c8" "zb_acrux_vp1_0_revb")
# VARIANTS=("ACTIPOD_4L" "ACTIPOD_4L_HIGHLY" "ACTIPOD_4L_HIGHLY_4G" "ACTIPOD_15L_4G" "ACTIPOD_15L_HYBRID" "ACTIPOD_4L_444" "ACTIPOD_15L")
# GIT_COMMIT_ID=$(git rev-parse --short=6 HEAD)


# # 🧹 Clean old builds
# BUILD_DIR="build_custom"
# echo -e "${YELLOW}🧹 Cleaning previous builds...${RESET}"
# rm -rf "$BUILD_DIR"
# mkdir -p "$BUILD_DIR"

# # 📥 Select boards
# echo -e "${YELLOW}Available Boards:${RESET}"
# for i in "${!BOARDS[@]}"; do
#     echo "$((i+1)). ${BOARDS[$i]}"
# done
# read -p "👉 Enter board numbers (comma-separated): " board_input

# IFS=',' read -ra BOARD_INDICES <<< "$board_input"
# SELECTED_BOARDS=()
# for index in "${BOARD_INDICES[@]}"; do
#     SELECTED_BOARDS+=("${BOARDS[$((index-1))]}")
# done

# # 🎯 Select variants per board
# declare -A BOARD_VARIANT_MAP

# for board in "${SELECTED_BOARDS[@]}"; do
#     echo -e "\n${YELLOW}Select variants for $board:${RESET}"
#     for i in "${!VARIANTS[@]}"; do
#         echo "$((i+1)). ${VARIANTS[$i]}"
#     done
#     read -p "👉 Enter variant numbers for $board (comma-separated): " variant_input
#     IFS=',' read -ra VARIANT_INDICES <<< "$variant_input"
#     variants_for_board=()
#     for index in "${VARIANT_INDICES[@]}"; do
#         variants_for_board+=("${VARIANTS[$((index-1))]}")
#     done
#     BOARD_VARIANT_MAP["$board"]="${variants_for_board[*]}"
# done

# # 🧾 Save Git commit info
# echo -e "${YELLOW}🔍 Saving Git commit info...${RESET}"
# git log -1 > "$BUILD_DIR/commit_info.txt"

# # 🔁 Begin build process
# for board in "${SELECTED_BOARDS[@]}"; do
#     IFS=' ' read -ra variants <<< "${BOARD_VARIANT_MAP[$board]}"
#     for variant in "${variants[@]}"; do
#         OUT_DIR="$BUILD_DIR/$board/$variant"
#         echo -e "${YELLOW}🔨 Building for $board [$variant]...${RESET}"
#         mkdir -p "$OUT_DIR"
        


#         export VERSION=D4_$(date +%d-%m-%y)_zephyr_commit_#$(git rev-parse --short=6 HEAD)
#         west build -p -c -b "$board" zb_actipod_app -d "$OUT_DIR"  -- -DCONFIG_"$variant"=y -DCONFIG_APP_SYS_VERSION=\"$(echo $VERSION)\" -DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE=\"bootloader/mcuboot/zb-ed25519.pem\" > "$OUT_DIR/build_log.txt" 2>&1

        
#         # 🧹 Clean up: keep only binary outputs and log
#         find "$OUT_DIR/zephyr" -type f ! \( -name "*.hex" -o -name "*.bin" -o -name "*.elf" \) -delete

#         echo -e "${GREEN}✅ Done: $board [$variant]${RESET}"
#     done
# done

# echo -e "${GREEN}🎉 All builds complete! Check the '$BUILD_DIR' folder.${RESET}"



#!/bin/bash
set -e
trap 'echo -e "\e[31m❌ Error occurred. Exiting.\e[0m"' ERR

# 🎨 Colors
GREEN="\e[32m"
RED="\e[31m"
YELLOW="\e[33m"
RESET="\e[0m"

# 📋 Available options
BOARDS=("zb_tv_h743zi" "zb_tv_h723zi" "zb_bv_l433rc" "zb_bv_l151c8" "zb_acrux_vp1_0_revb")
VARIANTS=("ACTIPOD_4L" "ACTIPOD_4L_HIGHLY" "ACTIPOD_4L_HIGHLY_4G" "ACTIPOD_15L_4G" "ACTIPOD_15L_HYBRID" "ACTIPOD_4L_444" "ACTIPOD_15L")



# 🧹 Clean old builds
BUILD_DIR="build_custom"
echo -e "${YELLOW}🧹 Cleaning previous builds...${RESET}"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# 📥 Select boards
echo -e "${YELLOW}Available Boards:${RESET}"
for i in "${!BOARDS[@]}"; do
    echo "$((i+1)). ${BOARDS[$i]}"
done
read -p "👉 Enter board numbers (comma-separated): " board_input

IFS=',' read -ra BOARD_INDICES <<< "$board_input"
SELECTED_BOARDS=()
for index in "${BOARD_INDICES[@]}"; do
    SELECTED_BOARDS+=("${BOARDS[$((index-1))]}")
done

# 🎯 Select variants per board
declare -A BOARD_VARIANT_MAP

for board in "${SELECTED_BOARDS[@]}"; do
    echo -e "\n${YELLOW}Select variants for $board:${RESET}"
    for i in "${!VARIANTS[@]}"; do
        echo "$((i+1)). ${VARIANTS[$i]}"
    done
    read -p "👉 Enter variant numbers for $board (comma-separated): " variant_input
    IFS=',' read -ra VARIANT_INDICES <<< "$variant_input"
    variants_for_board=()
    for index in "${VARIANT_INDICES[@]}"; do
        variants_for_board+=("${VARIANTS[$((index-1))]}")
    done
    BOARD_VARIANT_MAP["$board"]="${variants_for_board[*]}"
done

# 🧾 Save Git commit info
echo -e "${YELLOW}🔍 Saving Git commit info...${RESET}"
git log -1 > "$BUILD_DIR/commit_info.txt"

# 🔁 Begin build process
for board in "${SELECTED_BOARDS[@]}"; do
    IFS=' ' read -ra variants <<< "${BOARD_VARIANT_MAP[$board]}"
    for variant in "${variants[@]}"; do
        OUT_DIR="$BUILD_DIR/$board/$variant"
        echo -e "${YELLOW}🔨 Building for $board [$variant]...${RESET}"
        mkdir -p "$OUT_DIR"


        export VERSION=D4_$(date +%d-%m-%y)_zephyr_commit_#$(git rev-parse --short=6 HEAD)
        west build -p -c -b "$board" zb_actipod_app -d "$OUT_DIR"  -- -DCONFIG_"$variant"=y -DCONFIG_APP_SYS_VERSION=\"$(echo $VERSION)\" -DCONFIG_MCUBOOT_SIGNATURE_KEY_FILE=\"bootloader/mcuboot/zb-ed25519.pem\" > "$OUT_DIR/build_log.txt" 2>&1


        # 🧹 Clean up: keep only binary outputs and log
        find "$OUT_DIR/zephyr" -type f ! \( -name "*.hex" -o -name "*.bin" -o -name "*.elf" \) -delete

        # Move the relevant binary files to the output directory
        mkdir -p "$OUT_DIR/binaries"
        mv "$OUT_DIR/zephyr"/*.hex "$OUT_DIR/binaries/" 2>/dev/null || true
        mv "$OUT_DIR/zephyr"/*.bin "$OUT_DIR/binaries/" 2>/dev/null || true
        mv "$OUT_DIR/zephyr"/*.elf "$OUT_DIR/binaries/" 2>/dev/null || true
        mv "$OUT_DIR/build_log.txt" "$OUT_DIR/binaries/" 2>/dev/null || true

        # Remove the intermediate "zephyr" directory
        rm -rf "$OUT_DIR/zephyr"
        rm -rf "$OUT_DIR/app"
        rm -rf "$OUT_DIR/CMakeFiles"
        rm -rf "$OUT_DIR/Kconfig"
        rm -rf "$OUT_DIR/modules"
        rm -f "$OUT_DIR/build.ninja"
        rm -f "$OUT_DIR/CMakeCache.txt"
        rm -f "$OUT_DIR/cmake_install.cmake"
        rm -f "$OUT_DIR/compile_commands.json"
        rm -f "$OUT_DIR/zephyr_modules.txt"
        rm -f "$OUT_DIR/zephyr_settings.txt"



        echo -e "${GREEN}✅ Done: $board [$variant]${RESET}"
    done
done

echo -e "${GREEN}🎉 All builds complete! Check the '$BUILD_DIR' folder.${RESET}"
