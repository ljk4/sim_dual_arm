# Interactive prompt and terminal title for every project container.
#
# Each container declares its own name through CONTAINER_NAME in its
# configs/container.env (runtime / simulation / learning).  Ubuntu's skeleton
# ~/.bashrc resets PS1 after this file runs, so the prompt is applied from
# PROMPT_COMMAND, which the skeleton never touches.
shopt -s checkwinsize

if [ -n "${CONTAINER_NAME:-}" ]; then
  PROMPT_COMMAND='PS1="\[\e[1;32m\]${CONTAINER_NAME}\[\e[0m\]:\w\$ "; printf "\033]0;%s\007" "${CONTAINER_NAME}"'
fi
